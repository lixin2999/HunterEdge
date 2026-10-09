# -*- coding: utf-8 -*-
"""hunter_kafka 配置装配单元测试（对应 HunterCore 接入包 kafka.properties 模板）。

运行：
    cd ~/HunterEdge && colcon test --packages-select hunter_kafka && colcon test-result --verbose
或：pytest src/hunter_common/hunter_kafka/test/test_config.py
"""
import os

import pytest

from hunter_kafka.client import TopicNameError, topic_name
from hunter_kafka.config import (KafkaConfigError, build_client_conf,
                                  load_properties, parse_jaas_credentials,
                                  redact, strip_meta)

# 与 HunterCore 接入包（HUNTER-001 bundle）逐字对齐的模板文本
TEMPLATE = """# HunterCore 车端接入配置 —— vehicle_id=HUNTER-001
bootstrap.servers=192.168.31.35:9093
security.protocol=SASL_SSL
sasl.mechanism=SCRAM-SHA-512
sasl.jaas.config=org.apache.kafka.common.security.scram.ScramLoginModule required \\
  username="HUNTER-001" \\
  password="{password}";

ssl.endpoint.identification.algorithm=https

# 生产者基准参数（对齐 contracts/kafka/topics.yaml#producer_defaults）
linger.ms=5
batch.size=16384
retries=3
compression.type=lz4
acks=1                  # telemetry 高频；event/command/ota_* 请按 Topic 单独设置 acks=all

# 消费者基准
enable.auto.commit=false
auto.offset.reset=earliest
isolation.level=read_committed
"""


@pytest.fixture()
def bundle(tmp_path):
    """构造一个最小接入包目录（证书内容不重要，装配只看文件是否存在）。"""
    for name in ('ca-cert.pem', 'client-cert.pem', 'client-key.pem', 'kafka-client.p12'):
        (tmp_path / name).write_text('placeholder')
    props_file = tmp_path / 'kafka.properties'
    props_file.write_text(TEMPLATE.format(password='s3cr3t#pw'))
    return tmp_path


def _props(bundle, password='s3cr3t#pw'):
    path = bundle / 'kafka.properties'
    path.write_text(TEMPLATE.format(password=password))
    return load_properties(str(path))


def test_inline_comment_is_stripped_from_value(bundle):
    """java.util.Properties 会把 ' # 注释' 当作值的一部分，librdkafka 会拒绝该值。"""
    props = _props(bundle)
    assert props['acks'] == '1'
    assert props['linger.ms'] == '5'
    assert props['bootstrap.servers'] == '192.168.31.35:9093'


def test_jaas_multiline_is_merged_and_parsed(bundle):
    props = _props(bundle)
    user, pwd = parse_jaas_credentials(props['sasl.jaas.config'])
    assert user == 'HUNTER-001'
    # 口令中的 '#' 前无空白，不能被当作注释剥掉
    assert pwd == 's3cr3t#pw'


def test_build_conf_maps_java_keys_to_librdkafka(bundle):
    conf = build_client_conf(_props(bundle), bundle_dir=str(bundle),
                             client_id='HUNTER-001-telemetry', role='producer', acks='1')
    assert conf['sasl.username'] == 'HUNTER-001'
    assert conf['sasl.password'] == 's3cr3t#pw'
    assert conf['security.protocol'] == 'sasl_ssl'
    assert 'sasl.jaas.config' not in conf          # librdkafka 不认该键
    assert 'ssl.truststore.location' not in conf   # JKS 不可用，必须回落 PEM
    assert conf['ssl.ca.location'].endswith('ca-cert.pem')
    assert conf['ssl.certificate.location'].endswith('client-cert.pem')
    assert conf['ssl.key.location'].endswith('client-key.pem')
    assert conf['ssl.endpoint.identification.algorithm'] == 'https'
    assert conf['compression.type'] == 'lz4'
    assert conf['acks'] == '1'


def test_placeholder_password_fails_fast(bundle):
    """SCRAM 口令未填必须启动即报错，而不是运行期 SASL 反复失败。"""
    with pytest.raises(KafkaConfigError):
        build_client_conf(_props(bundle, password='<SCRAM_PASSWORD>'),
                          bundle_dir=str(bundle), role='producer')


def test_missing_client_certificate_fails(bundle):
    props = _props(bundle)
    with pytest.raises(KafkaConfigError):
        build_client_conf(props, bundle_dir=str(bundle / 'nowhere'), role='producer')


def test_p12_fallback_when_pem_absent(bundle):
    os.remove(bundle / 'client-cert.pem')
    os.remove(bundle / 'client-key.pem')
    props = _props(bundle)
    props['ssl.keystore.password'] = 'DWNhkDuPh4knJS6I3I7LWg'
    conf = build_client_conf(props, bundle_dir=str(bundle), role='producer')
    assert conf['ssl.keystore.location'].endswith('kafka-client.p12')
    assert conf['ssl.keystore.password'] == 'DWNhkDuPh4knJS6I3I7LWg'


def test_consumer_gets_consumer_side_keys(bundle):
    conf = build_client_conf(_props(bundle), bundle_dir=str(bundle), role='consumer')
    assert conf['enable.auto.commit'] == 'false'
    assert conf['auto.offset.reset'] == 'earliest'
    assert conf['isolation.level'] == 'read_committed'


def test_redact_and_strip_meta(bundle):
    conf = build_client_conf(_props(bundle), bundle_dir=str(bundle), role='producer')
    conf['__properties_path__'] = '/etc/hunter/kafka/kafka.properties'
    assert 's3cr3t#pw' not in str(redact(conf))
    assert '__properties_path__' not in strip_meta(conf)


def test_topic_name_contract():
    assert topic_name('HUNTER-001', 'command_result') == 'hunter.HUNTER-001.command_result'
    # remote_status 不在契约清单内（旧版 remote_agent 曾自发该 Topic）
    with pytest.raises(TopicNameError):
        topic_name('HUNTER-001', 'remote_status')
