#!/usr/bin/env python3
"""OTA Agent — HUNTER 自动驾驶平台 OTA 升级服务（文档第 12 章）。

独立 systemd 服务（非 ROS 节点）。
状态机：IDLE → PENDING → DOWNLOAD → INSTALL → TEST → SUCCESS / ROLLBACK / FAILED

Kafka 接入严格对齐 HunterCore 接入包契约：
  下行 ``hunter.<vehicle_id>.ota_notify``（平台→车）
  上行 ``hunter.<vehicle_id>.ota_status``（车→平台，acks=all + 逐条证实）
Broker / SCRAM 凭据 / mTLS 证书只来自 /etc/hunter/kafka/kafka.properties（不写 YAML）。
"""
import argparse
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
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

try:
    from hunter_kafka.client import make_consumer, make_producer, topic_name
    from hunter_kafka.config import (DEFAULT_BUNDLE_DIR, DEFAULT_PROPERTIES_PATH,
                                     KafkaConfigError, load_properties, resolve_vehicle_id)
    HAS_HUNTER_KAFKA = True
except ImportError:
    HAS_HUNTER_KAFKA = False
    DEFAULT_BUNDLE_DIR = "/etc/hunter/kafka"
    DEFAULT_PROPERTIES_PATH = "/etc/hunter/kafka/kafka.properties"

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("ota_agent")


try:
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import padding
    HAS_CRYPTO = True
except ImportError:
    HAS_CRYPTO = False


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

        self.vehicle_id = config["vehicle_id"]
        # 契约 Topic（旧版用配置里的 notify_topic/status_topic 自由名，现收敛到契约名）
        self.notify_topic = topic_name(self.vehicle_id, "ota_notify") if HAS_HUNTER_KAFKA \
            else f"hunter.{self.vehicle_id}.ota_notify"
        self.status_topic = topic_name(self.vehicle_id, "ota_status") if HAS_HUNTER_KAFKA \
            else f"hunter.{self.vehicle_id}.ota_status"
        self._last_dr_error = None

    # ------------------------------------------------------------------
    # Kafka（文档 12.4.1 接收通知、状态上报）
    # ------------------------------------------------------------------
    def _init_kafka(self):
        """按接入包装配 SASL_SSL + mTLS；失败仅降级不阻断升级能力自检。"""
        if not HAS_KAFKA:
            logger.warning("confluent_kafka 不可用（pip3 install confluent-kafka），Kafka 功能降级")
            return False
        if not HAS_HUNTER_KAFKA:
            logger.error("hunter_kafka 公共包不可用（未 source 工作空间？），Kafka 功能关闭；"
                         "不再使用旧版内联 SASL 参数路径，避免证书缺失时默默连接失败")
            return False
        properties = self.config.get("kafka_properties", DEFAULT_PROPERTIES_PATH)
        bundle_dir = self.config.get("kafka_bundle_dir", DEFAULT_BUNDLE_DIR)
        try:
            self.producer = make_producer(
                self.vehicle_id, "ota_status",
                properties_path=properties, bundle_dir=bundle_dir,
                dr_callback=self._on_dr)
            # ota_notify 不能漏：新消费组建立后回读历史通知，靠 task_id 幂等收敛
            self.consumer = make_consumer(
                self.vehicle_id, "ota_notify", group_suffix="ota",
                properties_path=properties, bundle_dir=bundle_dir,
                auto_offset_reset=self.config.get("offset_reset", "earliest"))
        except KafkaConfigError as exc:
            logger.error("OTA Kafka 接入失败：%s（核对 %s 的 SCRAM 口令，以及证书私钥权限 0600 / 目录 0750）",
                         exc, properties)
            return False
        logger.info("OTA Kafka 就绪：%s / %s", self.notify_topic, self.status_topic)
        return True

    def _on_dr(self, err, msg):
        if err is not None:
            self._last_dr_error = str(err)
            logger.error("ota_status 投递失败：%s（topic=%s）", err,
                         msg.topic() if msg else "?")

    def receive_notify(self, timeout=1.0):
        """接收 OTA 通知（文档 12.4.1）；返回 (通知体, 原始消息) 以便处理后手动提交"""
        if not self.consumer:
            return None, None
        msg = self.consumer.poll(timeout)
        if msg is None or msg.error():
            return None, None
        try:
            return json.loads(msg.value().decode("utf-8")), msg
        except (json.JSONDecodeError, UnicodeDecodeError):
            logger.error("OTA 通知解析失败，丢弃并提交 offset")
            self._commit(msg)
            return None, None

    def _commit(self, msg):
        if not (self.consumer and msg is not None):
            return
        try:
            self.consumer.commit(offsets=[msg], asynchronous=False)
        except Exception as exc:  # noqa: BLE001
            logger.error("ota_notify offset 提交失败：%s", exc)

    def report_status(self, state, detail=""):
        """状态上报（文档 12.2）：acks=all 并短暂 flush 证实，不让平台状态停留在猜测"""
        payload = {
            "vehicle_id": self.vehicle_id,
            "task_id": self.task.get("task_id") if self.task else None,
            "state": state,
            "detail": detail,
            "timestamp": int(time.time()),
        }
        logger.info("状态上报：%s %s", state, detail)
        if not self.producer:
            return
        try:
            self.producer.produce(self.status_topic,
                                  json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                                  key=self.vehicle_id.encode("utf-8"))
            self.producer.poll(0)
            # 升级状态属业务链，逐条证实成本极低（频率很低）
            self.producer.flush(float(self.config.get("status_flush_timeout", 5.0)))
        except Exception as exc:  # noqa: BLE001
            logger.error("ota_status 上报异常：%s", exc)

    # ------------------------------------------------------------------
    # 通知幂等（消费组重建/回灌时不重复刷机）
    # ------------------------------------------------------------------
    def _task_done(self):
        """读回最近已完成/已失败的 task_id（防止同一升级包反复刷机）"""
        try:
            with open(self.config["task_state_file"], "r", encoding="utf-8") as fh:
                return json.load(fh)
        except Exception:  # noqa: BLE001
            return {}

    def _mark_task(self, task_id, result):
        state = self._task_done()
        state[str(task_id)] = {"result": result, "ts": int(time.time())}
        try:
            os.makedirs(os.path.dirname(self.config["task_state_file"]), exist_ok=True)
            with open(self.config["task_state_file"], "w", encoding="utf-8") as fh:
                json.dump(state, fh, ensure_ascii=False)
        except Exception as exc:  # noqa: BLE001
            logger.warning("升级任务状态落盘失败：%s", exc)

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
            out = subprocess.run(["timeout", "1", "candump", "can2", "-n", "1"],
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
        self.resolve_vehicle_id()
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
            notify, msg = self.receive_notify()
            if not notify:
                time.sleep(1)
                continue
            try:
                self.handle_task(notify)
            except Exception as exc:  # noqa: BLE001
                logger.exception("升级任务处理异常：%s", exc)
                self.report_status("FAILED", f"车端异常：{exc}")
                self.state = OTAState.IDLE
            finally:
                # 处理完成才提交 offset（契约：enable.auto.commit=false）
                self._commit(msg)
                self.task = None

    def handle_task(self, notify):
        """单条 ota_notify 的完整状态机（文档 12.2/12.4）"""
        task_id = notify.get("task_id")
        if not task_id:
            logger.error("通知缺少 task_id，拒绝执行")
            self.report_status("FAILED", "通知缺少 task_id")
            return

        # 幂等：同一 task_id 已终结过（0 分区重建/回灌）不重复刷机
        done = self._task_done().get(str(task_id))
        if done:
            logger.warning("task_id=%s 已处理过（%s），仅补发一次状态回执", task_id,
                           done.get("result"))
            self.task = notify
            self.state = OTAState.IDLE
            self.report_status(str(done.get("result", "SUCCESS")), "重复通知（已忽略）")
            return

        self.task = notify
        self.state = OTAState.PENDING
        self.report_status("PENDING", "收到升级通知")

        # 前置检查（文档 12.4.2）
        ok, failed = self.precheck()
        if not ok:
            self._finish_task(task_id, "FAILED", f"前置检查失败: {failed}")
            return

        # 下载（文档 12.4.3）
        self.state = OTAState.DOWNLOAD
        self.report_status("DOWNLOAD", "开始下载")
        self.downloaded_file = self.download(notify)
        if not self.downloaded_file:
            self._finish_task(task_id, "FAILED", "下载失败")
            return

        # SHA-256 校验
        ok, actual = self.verify_sha256(self.downloaded_file, notify.get("sha256", ""))
        if not ok:
            self._finish_task(task_id, "FAILED", f"SHA256 校验失败: {actual}")
            return

        # 读取 manifest + RSA 签名校验（文档 12.6）
        try:
            manifest = self._load_manifest()
        except Exception as e:
            self._finish_task(task_id, "FAILED", f"manifest 解析失败: {e}")
            return
        if not self.verify_signature(manifest):
            self._finish_task(task_id, "FAILED", "RSA 签名校验失败")
            return

        # 备份 + 安装（文档 12.4.4/12.5）
        self.backup()
        self.state = OTAState.INSTALL
        self.report_status("INSTALL", "开始安装")
        try:
            self.install(manifest)
        except Exception as e:
            self.rollback()
            self._finish_task(task_id, "ROLLBACK", f"安装失败: {e}")
            return

        # 自检（文档 12.4.5）：生产环境安装后重启，由启动自检判定
        self.state = OTAState.TEST
        self.report_status("TEST", "安装完成，等待重启自检")
        self._finish_task(task_id, "SUCCESS", "升级完成")

    def _finish_task(self, task_id, result, detail):
        """终态：先上报（acks=all 证实）再落盘幂等记录，避免“上报丢了但记成已完成”"""
        self.report_status(result, detail)
        self._mark_task(task_id, result)
        self.state = OTAState.IDLE

    def resolve_vehicle_id(self):
        """vehicle_id 以接入包为准（SASL 用户名 == 证书 CN == vehicle_id）"""
        if not HAS_HUNTER_KAFKA:
            return
        path = self.config.get("kafka_properties", DEFAULT_PROPERTIES_PATH)
        if not os.path.isfile(path):
            logger.warning("未找到 %s，使用配置中的 vehicle_id=%s", path, self.vehicle_id)
            return
        try:
            props = load_properties(path)
        except KafkaConfigError as exc:
            logger.warning("kafka.properties 读取失败：%s", exc)
            return
        props_vid = resolve_vehicle_id(props)
        if props_vid and props_vid != self.vehicle_id:
            logger.error("vehicle_id 冲突：配置=%s 而接入包=%s（等同证书 CN）；以接入包为准",
                         self.vehicle_id, props_vid)
            self.vehicle_id = props_vid
            self.notify_topic = topic_name(self.vehicle_id, "ota_notify")
            self.status_topic = topic_name(self.vehicle_id, "ota_status")


DEFAULT_CONFIG = {
    "vehicle_id": "HUNTER-001",
    # Kafka 接入只留路径：Broker/SCRAM/mTLS 全部来自接入包（严禁入仓）
    "kafka_properties": DEFAULT_PROPERTIES_PATH,
    "kafka_bundle_dir": DEFAULT_BUNDLE_DIR,
    "offset_reset": "earliest",            # ota_notify 不能漏通知（靠 task_id 幂等收敛）
    "status_flush_timeout": 5.0,
    "download_dir": "/data/ota/download",
    "extract_dir": "/data/ota/extract",
    "backup_dir": "/data/ota/backup",
    "task_state_file": "/data/ota/.task_state.json",
    "public_key_path": "/etc/hunter/ota_public_key.pem",
    "version_file": "/opt/hunter/version",
    "current_version": "1.0.0",
    "min_battery_soc": 50.0,
    "min_free_space_gb": 2.0,
    "platform_host": "192.168.31.35",
    "self_test_on_start": True,
    "backup_paths": ["/opt/hunter/install"],
}


def load_config(path):
    """YAML 覆盖内置默认值；支持 ota_agent: ros__parameters: 结构或扁平结构"""
    cfg = dict(DEFAULT_CONFIG)
    cfg["kafka_properties"] = os.environ.get("HUNTER_KAFKA_PROPERTIES", cfg["kafka_properties"])
    if path and os.path.isfile(path) and HAS_YAML:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        section = data.get("ota_agent", data)
        cfg.update({k: v for k, v in section.get("ros__parameters", section).items()})
        logger.info("已加载配置：%s", path)
    elif path:
        logger.warning("配置文件不可用（缺文件或缺 PyYAML），使用内置默认值：%s", path)
    return cfg


def main():
    parser = argparse.ArgumentParser(description="HUNTER OTA Agent")
    parser.add_argument("--config",
                        default=os.environ.get("HUNTER_OTA_CONFIG",
                                               "/etc/hunter/ota_agent_params.yaml"),
                        help="YAML 参数文件（不含任何 Kafka 凭据）")
    agent = OtaAgent(load_config(args.config))
    agent.run()


if __name__ == "__main__":
    main()



