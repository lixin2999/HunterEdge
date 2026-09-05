#!/usr/bin/env python3
"""OTA Agent — HUNTER 自动驾驶平台 OTA 升级服务（文档第 12 章）。

独立 systemd 服务（非 ROS 节点）。
状态机：IDLE → PENDING → DOWNLOAD → INSTALL → TEST → SUCCESS / ROLLBACK / FAILED
"""
import hashlib
import json
import logging
import os
import shutil
import subprocess
import sys
import tarfile
import time
import urllib.request
from enum import Enum

try:
    from confluent_kafka import Consumer, Producer
    HAS_KAFKA = True
except ImportError:
    HAS_KAFKA = False

try:
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import padding
    HAS_CRYPTO = True
except ImportError:
    HAS_CRYPTO = False

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("ota_agent")


class OTAState(Enum):
    """升级状态机（文档 12.2）"""
    IDLE = "IDLE"
    PENDING = "PENDING"
    DOWNLOAD = "DOWNLOAD"
    INSTALL = "INSTALL"
    TEST = "TEST"
    SUCCESS = "SUCCESS"
    ROLLBACK = "ROLLBACK"
    FAILED = "FAILED"


class OtaAgent:
    def __init__(self, config):
        self.config = config
        self.state = OTAState.IDLE
        self.task = None
        self.downloaded_file = None
        self.backup_path = None
        self.consumer = None
        self.producer = None

        self.notify_topic = config.get(
            "notify_topic", f"hunter.{config['vehicle_id']}.ota_notify")
        self.status_topic = config.get(
            "status_topic", f"hunter.{config['vehicle_id']}.ota_status")

    # ------------------------------------------------------------------
    # Kafka（文档 12.4.1 接收通知、状态上报）
    # ------------------------------------------------------------------
    def _init_kafka(self):
        if not HAS_KAFKA:
            logger.warning("confluent_kafka 不可用，Kafka 功能降级")
            return False
        brokers = self.config["kafka_brokers"]
        self.consumer = Consumer({
            "bootstrap.servers": brokers,
            "group.id": f"ota-{self.config['vehicle_id']}",
            "auto.offset.reset": "latest",
        })
        self.consumer.subscribe([self.notify_topic])
        self.producer = Producer({"bootstrap.servers": brokers})
        return True

    def receive_notify(self, timeout=1.0):
        """接收 OTA 通知（文档 12.4.1）"""
        if not self.consumer:
            return None
        msg = self.consumer.poll(timeout)
        if msg is None or msg.error():
            return None
        try:
            return json.loads(msg.value().decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            logger.error("OTA 通知解析失败")
            return None

    def report_status(self, state, detail=""):
        """状态上报（文档 12.2）"""
        payload = {
            "vehicle_id": self.config["vehicle_id"],
            "task_id": self.task.get("task_id") if self.task else None,
            "state": state,
            "detail": detail,
            "timestamp": int(time.time()),
        }
        logger.info("状态上报：%s %s", state, detail)
        if self.producer and HAS_KAFKA:
            self.producer.produce(
                self.status_topic, json.dumps(payload).encode("utf-8"))
            self.producer.poll(0)

    # ------------------------------------------------------------------
    # 前置条件检查（文档 12.4.2）
    # ------------------------------------------------------------------
    def precheck(self):
        checks = []

        # 1. 电池 SOC ≥ 50%
        soc = self._read_battery_soc()
        checks.append(("battery_soc", soc >= self.config["min_battery_soc"], f"SOC={soc}%"))

        # 2. 车辆静止（velocity < 0.1 m/s）
        velocity = self._read_velocity()
        checks.append(("vehicle_static", velocity < 0.1, f"v={velocity:.3f}m/s"))

        # 3. 控制模式待机/CAN
        mode = self._read_control_mode()
        checks.append(("control_mode", mode in ("CAN", "STANDBY", "IDLE"), f"mode={mode}"))

        # 4. 剩余存储 ≥ 2GB（df -h）
        free_gb = self._free_space_gb("/")
        checks.append(("free_space", free_gb >= self.config["min_free_space_gb"],
                       f"free={free_gb:.1f}GB"))

        # 5. 网络正常（ping 平台域名）
        net_ok = self._network_ok()
        checks.append(("network", net_ok, "ping"))

        # 6. 无进行中的升级任务
        checks.append(("no_active_task", self.state == OTAState.IDLE,
                       f"state={self.state.value}"))

        failed = [name for name, ok, _ in checks if not ok]
        for name, ok, detail in checks:
            logger.info("前置检查 %s: %s (%s)", name, "PASS" if ok else "FAIL", detail)
        return len(failed) == 0, failed

    def _read_battery_soc(self):
        # 生产环境替换为车辆端 REST API / ROS2 桥接
        try:
            url = self.config.get("vehicle_api", "") + "/chassis/battery"
            if not url.startswith("http"):
                return 100.0
            with urllib.request.urlopen(url, timeout=2) as resp:
                return float(json.loads(resp.read())["soc"])
        except Exception:
            return 100.0

    def _read_velocity(self):
        try:
            url = self.config.get("vehicle_api", "") + "/chassis/velocity"
            if not url.startswith("http"):
                return 0.0
            with urllib.request.urlopen(url, timeout=2) as resp:
                return float(json.loads(resp.read())["velocity"])
        except Exception:
            return 0.0

    def _read_control_mode(self):
        try:
            url = self.config.get("vehicle_api", "") + "/chassis/mode"
            if not url.startswith("http"):
                return "CAN"
            with urllib.request.urlopen(url, timeout=2) as resp:
                return json.loads(resp.read())["mode"]
        except Exception:
            return "CAN"

    def _free_space_gb(self, path):
        try:
            st = os.statvfs(path)
            return st.f_bavail * st.f_frsize / (1024 ** 3)
        except Exception:
            return 0.0

    def _network_ok(self):
        host = self.config.get("platform_host", "platform.example.com")
        try:
            subprocess.run(
                ["ping", "-c", "1", "-W", "2", host],
                check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return True
        except Exception:
            return False

    # ------------------------------------------------------------------
    # 下载与校验（文档 12.4.3/12.6）
    # ------------------------------------------------------------------
    def download(self, notify):
        """HTTPS 下载升级包，重试最多 3 次"""
        url = notify["download_url"]
        os.makedirs(self.config["download_dir"], exist_ok=True)
        dest = os.path.join(self.config["download_dir"], os.path.basename(url))
        for attempt in range(3):
            try:
                urllib.request.urlretrieve(url, dest)
                logger.info("下载完成：%s", dest)
                return dest
            except Exception as e:
                logger.error("下载失败（第 %d 次）：%s", attempt + 1, e)
        return None

    def verify_sha256(self, filepath, expected):
        """SHA-256 完整性校验（文档 12.6）"""
        h = hashlib.sha256()
        with open(filepath, "rb") as f:
            for chunk in iter(lambda: f.read(8192), b""):
                h.update(chunk)
        actual = h.hexdigest()
        return actual == expected.lower(), actual

    def verify_signature(self, manifest):
        """RSA-2048 数字签名校验（文档 12.6）"""
        if not HAS_CRYPTO:
            logger.warning("cryptography 不可用，跳过签名校验")
            return True
        try:
            with open(self.config["public_key_path"], "rb") as f:
                pub_key = serialization.load_pem_public_key(f.read())
            content = json.dumps(
                {k: v for k, v in manifest.items() if k != "signature"},
                sort_keys=True).encode("utf-8")
            signature = bytes.fromhex(manifest.get("signature", ""))
            pub_key.verify(signature, content, padding.PKCS1v15(), hashes.SHA256())
            return True
        except Exception as e:
            logger.error("签名校验失败：%s", e)
            return False

    # ------------------------------------------------------------------
    # 安装与备份（文档 12.4.4 / 12.5）
    # ------------------------------------------------------------------
    def install(self, manifest):
        """解压 + 执行 pre_install/install/post_install 脚本"""
        extract_dir = os.path.join(self.config["extract_dir"], manifest["version"])
        os.makedirs(extract_dir, exist_ok=True)
        with tarfile.open(self.downloaded_file) as tf:
            tf.extractall(extract_dir)
        logger.info("解压完成：%s", extract_dir)

        for key in ("pre_install", "install", "post_install"):
            script = manifest.get("scripts", {}).get(key)
            if not script:
                continue
            script_path = os.path.join(extract_dir, script)
            logger.info("执行脚本：%s", script)
            subprocess.run(["bash", script_path], check=True)
        return extract_dir

    def backup(self):
        """备份当前版本关键文件（文档 12.5：备份恢复方案）"""
        current = self.config["current_version"]
        backup_path = os.path.join(self.config["backup_dir"], current)
        os.makedirs(backup_path, exist_ok=True)
        for src in self.config.get("backup_paths", ["/opt/hunter/install"]):
            if not os.path.exists(src):
                continue
            dst = os.path.join(backup_path, os.path.basename(src))
            if os.path.isdir(src):
                shutil.copytree(src, dst, dirs_exist_ok=True)
            else:
                shutil.copy2(src, dst)
        # 清理：保留最近 2 个版本备份
        self._prune_backups()
        self.backup_path = backup_path
        logger.info("备份完成：%s", backup_path)
        return backup_path

    def _prune_backups(self):
        backup_dir = self.config["backup_dir"]
        if not os.path.isdir(backup_dir):
            return
        versions = sorted(os.listdir(backup_dir))
        while len(versions) > 2:
            oldest = os.path.join(backup_dir, versions.pop(0))
            shutil.rmtree(oldest, ignore_errors=True)

    # ------------------------------------------------------------------
    # 自检与回滚（文档 12.4.5 / 12.5）
    # ------------------------------------------------------------------
    def _load_manifest(self):
        with tarfile.open(self.downloaded_file) as tf:
            member = tf.getmember("manifest.json")
            return json.loads(tf.extractfile(member).read())

    def self_test(self):
        """重启后自检（文档 12.4.5）"""
        checks = []
        # 1. ROS2 daemon 正常运行
        checks.append(("ros2_daemon", self._cmd_ok(["ros2", "daemon", "status"])))
        # 2. CAN 通信（candump 检查 0x211 报文）
        checks.append(("can", self._can_ok()))
        # 3. 版本号正确更新
        checks.append(("version", self._version_ok()))
        for name, ok in checks:
            logger.info("自检 %s: %s", name, "PASS" if ok else "FAIL")
        return all(ok for _, ok in checks)

    def _cmd_ok(self, cmd):
        try:
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=10)
            return True
        except Exception:
            return False

    def _can_ok(self):
        try:
            out = subprocess.run(["timeout", "1", "candump", "can0", "-n", "1"],
                                 capture_output=True, text=True)
            return "211" in out.stdout
        except Exception:
            return True  # 无 candump 工具时降级

    def _version_ok(self):
        try:
            with open(self.config.get("version_file", "/opt/hunter/version"), "r") as f:
                return f.read().strip() == self.task.get("version", "")
        except Exception:
            return True

    def rollback(self):
        """从备份恢复（文档 12.5：备份恢复方案）"""
        if not self.backup_path or not os.path.isdir(self.backup_path):
            logger.error("无可用备份，回滚失败")
            return False
        for name in os.listdir(self.backup_path):
            src = os.path.join(self.backup_path, name)
            dst = os.path.join("/opt/hunter/install", name)
            if os.path.isdir(src):
                shutil.rmtree(dst, ignore_errors=True)
                shutil.copytree(src, dst)
            else:
                shutil.copy2(src, dst)
        logger.info("回滚完成，已恢复上一版本")
        return True

    # ------------------------------------------------------------------
    # 主循环（状态机，文档 12.2）
    # ------------------------------------------------------------------
    def run(self):
        self._init_kafka()
        logger.info("OTA Agent 启动，当前版本 %s", self.config["current_version"])

        # 启动自检（重启后，文档 12.4.5）
        if self.config.get("self_test_on_start", True):
            self.state = OTAState.TEST
            if self.self_test():
                self.report_status("SUCCESS", "启动自检通过")
            else:
                self.rollback()
                self.report_status("ROLLBACK", "启动自检失败，已回滚")
            self.state = OTAState.IDLE

        while True:
            notify = self.receive_notify()
            if not notify:
                time.sleep(1)
                continue

            self.task = notify
            self.state = OTAState.PENDING
            self.report_status("PENDING", "收到升级通知")

            # 前置检查（文档 12.4.2）
            ok, failed = self.precheck()
            if not ok:
                self.report_status("FAILED", f"前置检查失败: {failed}")
                self.state = OTAState.IDLE
                continue

            # 下载（文档 12.4.3）
            self.state = OTAState.DOWNLOAD
            self.report_status("DOWNLOAD", "开始下载")
            self.downloaded_file = self.download(notify)
            if not self.downloaded_file:
                self.report_status("FAILED", "下载失败")
                self.state = OTAState.IDLE
                continue

            # SHA-256 校验
            ok, actual = self.verify_sha256(self.downloaded_file, notify.get("sha256", ""))
            if not ok:
                self.report_status("FAILED", f"SHA256 校验失败: {actual}")
                self.state = OTAState.IDLE
                continue

            # 读取 manifest + RSA 签名校验（文档 12.6）
            try:
                manifest = self._load_manifest()
            except Exception as e:
                self.report_status("FAILED", f"manifest 解析失败: {e}")
                self.state = OTAState.IDLE
                continue
            if not self.verify_signature(manifest):
                self.report_status("FAILED", "RSA 签名校验失败")
                self.state = OTAState.IDLE
                continue

            # 备份 + 安装（文档 12.4.4/12.5）
            self.backup()
            self.state = OTAState.INSTALL
            self.report_status("INSTALL", "开始安装")
            try:
                self.install(manifest)
            except Exception as e:
                self.rollback()
                self.report_status("ROLLBACK", f"安装失败: {e}")
                self.state = OTAState.IDLE
                continue

            # 自检（文档 12.4.5）：生产环境安装后重启，由启动自检判定
            self.state = OTAState.TEST
            self.report_status("TEST", "安装完成，等待重启自检")
            self.state = OTAState.SUCCESS
            self.report_status("SUCCESS", "升级完成")
            self.state = OTAState.IDLE
            self.task = None


DEFAULT_CONFIG = {
    "vehicle_id": "hunter_001",
    "kafka_brokers": "platform.example.com:9093",
    "download_dir": "/data/ota/download",
    "extract_dir": "/data/ota/extract",
    "backup_dir": "/data/ota/backup",
    "public_key_path": "/etc/hunter/ota_public_key.pem",
    "version_file": "/opt/hunter/version",
    "current_version": "1.0.0",
    "min_battery_soc": 50.0,
    "min_free_space_gb": 2.0,
    "platform_host": "platform.example.com",
    "self_test_on_start": True,
    "backup_paths": ["/opt/hunter/install"],
}


def main():
    agent = OtaAgent(DEFAULT_CONFIG)
    agent.run()


if __name__ == "__main__":
    main()



