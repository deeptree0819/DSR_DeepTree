#!/usr/bin/env python3
"""
nlp_node.py — 자연어 처리 + 행동 매핑 노드

/stt_result (String) 구독
  → 방향 이동(jog) 우선 파싱
  → 키워드 기반 명령 파싱 (keyword_map.yaml)
  → 기어 번호 파싱 (task_id: 0=전체, 1~4=특정 기어)
  → 위글 옵션 파싱 (use_wiggle: 기본 true)
  → /robot_command (VoiceCommand) 퍼블리시
  → /tts_input     (String)       퍼블리시

음성 예시:
  "앞으로"               → jog +x 5cm
  "뒤로 3센티"           → jog -x 3cm
  "왼쪽으로 10밀리"      → jog +y 1cm
  "위로 5센티"           → jog +z 5cm
  "기어 1번 조립"        → pickplace task_id=1
  "두번째 기어 실행"     → pickplace task_id=2
  "위글없이 전체실행"    → pickplace use_wiggle=False
"""

import os
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from stt_robot_interfaces.msg import VoiceCommand

try:
    import yaml
except ImportError:
    yaml = None


# ════════════════════════════════════════════════════
#   기본 키워드 → 명령 매핑
# ════════════════════════════════════════════════════
DEFAULT_KEYWORD_MAP: dict[str, int] = {
    # HOME
    "홈":           VoiceCommand.CMD_HOME,
    "home":         VoiceCommand.CMD_HOME,
    "홈으로":       VoiceCommand.CMD_HOME,
    "처음":         VoiceCommand.CMD_HOME,
    "초기":         VoiceCommand.CMD_HOME,
    "복귀":         VoiceCommand.CMD_HOME,
    # PICK
    "픽":           VoiceCommand.CMD_PICK,
    "집어":         VoiceCommand.CMD_PICK,
    "잡아":         VoiceCommand.CMD_PICK,
    "pick":         VoiceCommand.CMD_PICK,
    "들어":         VoiceCommand.CMD_PICK,
    # PLACE
    "플레이스":     VoiceCommand.CMD_PLACE,
    "놓아":         VoiceCommand.CMD_PLACE,
    "내려놔":       VoiceCommand.CMD_PLACE,
    "place":        VoiceCommand.CMD_PLACE,
    "내려":         VoiceCommand.CMD_PLACE,
    # PICKPLACE
    "픽앤플레이스": VoiceCommand.CMD_PICKPLACE,
    "픽플레이스":   VoiceCommand.CMD_PICKPLACE,
    "pickandplace": VoiceCommand.CMD_PICKPLACE,
    "pickplace":    VoiceCommand.CMD_PICKPLACE,
    "집어서놔":     VoiceCommand.CMD_PICKPLACE,
    "가져다놔":     VoiceCommand.CMD_PICKPLACE,
    "조립":         VoiceCommand.CMD_PICKPLACE,
    "조립해":       VoiceCommand.CMD_PICKPLACE,
    "전체실행":     VoiceCommand.CMD_PICKPLACE,
    "실행":         VoiceCommand.CMD_PICKPLACE,
    # STOP
    "정지":         VoiceCommand.CMD_STOP,
    "멈춰":         VoiceCommand.CMD_STOP,
    "스톱":         VoiceCommand.CMD_STOP,
    "stop":         VoiceCommand.CMD_STOP,
    "취소":         VoiceCommand.CMD_STOP,
    "중지":         VoiceCommand.CMD_STOP,
    "그만":         VoiceCommand.CMD_STOP,
    # GRIPPER
    "그리퍼열어":   VoiceCommand.CMD_GRIPPER_OPEN,
    "그리퍼오픈":   VoiceCommand.CMD_GRIPPER_OPEN,
    "열어":         VoiceCommand.CMD_GRIPPER_OPEN,
    "그리퍼닫아":   VoiceCommand.CMD_GRIPPER_CLOSE,
    "그리퍼클로즈": VoiceCommand.CMD_GRIPPER_CLOSE,
    "닫아":         VoiceCommand.CMD_GRIPPER_CLOSE,
}

# ── 방향 키워드 → DIR_* 상수 ─────────────────────────────
# 긴 표현이 앞에 오도록 정렬 (아래로 > 아래)
_DIR_KEYWORDS: list[tuple[str, int]] = [
    ("앞으로",     VoiceCommand.DIR_FORWARD),
    ("앞",         VoiceCommand.DIR_FORWARD),
    ("전진",       VoiceCommand.DIR_FORWARD),
    ("forward",    VoiceCommand.DIR_FORWARD),
    ("뒤로",       VoiceCommand.DIR_BACKWARD),
    ("뒤",         VoiceCommand.DIR_BACKWARD),
    ("후진",       VoiceCommand.DIR_BACKWARD),
    ("backward",   VoiceCommand.DIR_BACKWARD),
    ("왼쪽으로",   VoiceCommand.DIR_LEFT),
    ("왼쪽",       VoiceCommand.DIR_LEFT),
    ("왼으로",     VoiceCommand.DIR_LEFT),
    ("왼",         VoiceCommand.DIR_LEFT),
    ("left",       VoiceCommand.DIR_LEFT),
    ("오른쪽으로", VoiceCommand.DIR_RIGHT),
    ("오른쪽",     VoiceCommand.DIR_RIGHT),
    ("오른으로",   VoiceCommand.DIR_RIGHT),
    ("오른",       VoiceCommand.DIR_RIGHT),
    ("right",      VoiceCommand.DIR_RIGHT),
    ("위로",       VoiceCommand.DIR_UP),
    ("위",         VoiceCommand.DIR_UP),
    ("up",         VoiceCommand.DIR_UP),
    ("아래로",     VoiceCommand.DIR_DOWN),
    ("아래",       VoiceCommand.DIR_DOWN),
    ("down",       VoiceCommand.DIR_DOWN),
]

_DIR_NAME: dict[int, str] = {
    VoiceCommand.DIR_FORWARD:  "앞(+x)",
    VoiceCommand.DIR_BACKWARD: "뒤(-x)",
    VoiceCommand.DIR_LEFT:     "왼(+y)",
    VoiceCommand.DIR_RIGHT:    "오(-y)",
    VoiceCommand.DIR_UP:       "위(+z)",
    VoiceCommand.DIR_DOWN:     "아래(-z)",
}

# ── 기어 번호 한국어 서수 ────────────────────────────────
_KO_ORDINAL: dict[str, int] = {
    "첫번째": 1, "1번": 1, "일번": 1,
    "두번째": 2, "2번": 2, "이번": 2,
    "세번째": 3, "3번": 3, "삼번": 3,
    "네번째": 4, "4번": 4, "사번": 4,
}

# ── 위글 비활성화 키워드 ─────────────────────────────────
_NO_WIGGLE_KEYWORDS = ["위글없이", "위글빼고", "위글안해", "nowiggle"]

DEFAULT_OFFSET_M = 0.05   # 5cm

CMD_RESPONSE: dict[int, str] = {
    VoiceCommand.CMD_HOME:         "홈 자세로 이동합니다.",
    VoiceCommand.CMD_PICK:         "픽 동작을 시작합니다.",
    VoiceCommand.CMD_PLACE:        "플레이스 동작을 시작합니다.",
    VoiceCommand.CMD_PICKPLACE:    "픽 앤 플레이스를 시작합니다.",
    VoiceCommand.CMD_STOP:         "동작을 중지합니다.",
    VoiceCommand.CMD_GRIPPER_OPEN:  "그리퍼를 엽니다.",
    VoiceCommand.CMD_GRIPPER_CLOSE: "그리퍼를 닫습니다.",
    VoiceCommand.CMD_JOG:          "이동합니다.",
    VoiceCommand.CMD_UNKNOWN:      "명령을 인식하지 못했습니다.",
}

CMD_NAME: dict[int, str] = {
    VoiceCommand.CMD_HOME:         "HOME",
    VoiceCommand.CMD_PICK:         "PICK",
    VoiceCommand.CMD_PLACE:        "PLACE",
    VoiceCommand.CMD_PICKPLACE:    "PICKPLACE",
    VoiceCommand.CMD_STOP:         "STOP",
    VoiceCommand.CMD_GRIPPER_OPEN:  "GRIPPER_OPEN",
    VoiceCommand.CMD_GRIPPER_CLOSE: "GRIPPER_CLOSE",
    VoiceCommand.CMD_JOG:          "JOG",
    VoiceCommand.CMD_UNKNOWN:      "UNKNOWN",
}


class NlpNode(Node):

    def __init__(self):
        super().__init__('nlp_node')

        self.declare_parameter('keyword_map_path',     '')
        self.declare_parameter('confidence_threshold', 0.0)

        map_path   = self.get_parameter('keyword_map_path').value
        self._conf = self.get_parameter('confidence_threshold').value

        self._keyword_map = self._load_keyword_map(map_path)
        self.get_logger().info(
            f'키워드 맵 로드 완료 — {len(self._keyword_map)}개 항목'
        )

        self._cmd_pub = self.create_publisher(VoiceCommand, '/robot_command', 10)
        self._tts_pub = self.create_publisher(String, '/tts_input', 10)
        self.create_subscription(String, '/stt_result', self._stt_cb, 10)

        self.get_logger().info('NLP 노드 준비 완료  /stt_result 구독 중')

    # ────────────────────────────────────────────
    #   키워드 맵 로드
    # ────────────────────────────────────────────
    def _load_keyword_map(self, path: str) -> dict[str, int]:
        if path and os.path.isfile(path) and yaml is not None:
            try:
                with open(path, encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                cmd_lookup = {
                    'home':          VoiceCommand.CMD_HOME,
                    'pick':          VoiceCommand.CMD_PICK,
                    'place':         VoiceCommand.CMD_PLACE,
                    'pickplace':     VoiceCommand.CMD_PICKPLACE,
                    'stop':          VoiceCommand.CMD_STOP,
                    'gripper_open':  VoiceCommand.CMD_GRIPPER_OPEN,
                    'gripper_close': VoiceCommand.CMD_GRIPPER_CLOSE,
                }
                result: dict[str, int] = {}
                for cmd_name, keywords in data.get('keyword_map', {}).items():
                    cmd_id = cmd_lookup.get(cmd_name.lower(), VoiceCommand.CMD_UNKNOWN)
                    for kw in keywords:
                        result[str(kw).lower()] = cmd_id
                self.get_logger().info(f'YAML 키워드 맵 로드: {path}')
                return result
            except Exception as e:
                self.get_logger().warn(f'YAML 로드 실패 ({e}), 내장 맵 사용')

        return dict(DEFAULT_KEYWORD_MAP)

    # ────────────────────────────────────────────
    #   STT 결과 콜백
    # ────────────────────────────────────────────
    def _stt_cb(self, msg: String):
        raw = msg.data
        log = self.get_logger()
        log.info(f'[NLP 수신] "{raw}"')

        # jog 방향 감지 최우선
        direction = self._parse_jog(raw)

        if direction != VoiceCommand.DIR_NONE:
            cmd_id     = VoiceCommand.CMD_JOG
            matched_kw = _DIR_NAME.get(direction, '')
            task_id    = 0
            use_wiggle = True
        else:
            cmd_id, matched_kw = self._parse_command(raw)
            task_id             = self._parse_task_id(raw)
            use_wiggle          = self._parse_use_wiggle(raw)

        cmd_name = CMD_NAME.get(cmd_id, 'UNKNOWN')

        if cmd_id == VoiceCommand.CMD_UNKNOWN:
            log.warn(f'매핑 실패 (무시): "{raw}"')
            return
        elif cmd_id == VoiceCommand.CMD_JOG:
            log.info(f'[매핑] JOG {_DIR_NAME.get(direction)}')
        else:
            log.info(
                f'[매핑] "{matched_kw}" → {cmd_name}  '
                f'task_id={task_id}  use_wiggle={use_wiggle}'
            )

        vc = VoiceCommand()
        vc.header.stamp    = self.get_clock().now().to_msg()
        vc.header.frame_id = 'nlp'
        vc.command         = cmd_id
        vc.raw_text        = raw
        vc.matched_keyword = matched_kw
        vc.confidence      = 1.0
        vc.task_id         = task_id
        vc.use_wiggle      = use_wiggle
        vc.direction       = direction

        vc.message         = cmd_name
        self._cmd_pub.publish(vc)

        # STOP 명령만 nlp에서 즉시 TTS — 나머지는 stt_pick_and_place가 처리
        if cmd_id == VoiceCommand.CMD_STOP:
            tts_msg = String()
            tts_msg.data = CMD_RESPONSE[VoiceCommand.CMD_STOP]
            self._tts_pub.publish(tts_msg)
            log.info(f'[TTS 전달] "{tts_msg.data}"')

    # ────────────────────────────────────────────
    #   방향 이동(Jog) 파싱
    #   반환: direction  — DIR_NONE 이면 jog 아님
    #   이동 거리는 stt_pick_and_place의 JOG_OFFSET 전역 변수로 결정
    # ────────────────────────────────────────────
    def _parse_jog(self, text: str) -> int:
        normalized = text.lower().strip().replace(' ', '')
        for kw, dir_val in _DIR_KEYWORDS:
            if kw in normalized:
                return dir_val
        return VoiceCommand.DIR_NONE

    # ────────────────────────────────────────────
    #   명령 파싱
    # ────────────────────────────────────────────
    def _parse_command(self, text: str) -> tuple[int, str]:
        normalized = text.lower().strip().replace(' ', '')

        for kw, cmd_id in self._keyword_map.items():
            if normalized == kw:
                return cmd_id, kw

        for kw in sorted(self._keyword_map, key=len, reverse=True):
            if kw in normalized:
                return self._keyword_map[kw], kw

        return VoiceCommand.CMD_UNKNOWN, ''

    # ────────────────────────────────────────────
    #   기어 번호 파싱 (task_id)
    # ────────────────────────────────────────────
    def _parse_task_id(self, text: str) -> int:
        normalized = text.lower().strip().replace(' ', '')

        m = re.search(r'기어\s*([1-4])', text)
        if m:
            return int(m.group(1))
        m = re.search(r'([1-4])\s*번\s*기어', text)
        if m:
            return int(m.group(1))
        m = re.search(r'([1-4])\s*번', text)
        if m:
            return int(m.group(1))

        for ko, num in _KO_ORDINAL.items():
            if ko in normalized:
                return num

        return 0

    # ────────────────────────────────────────────
    #   위글 옵션 파싱
    # ────────────────────────────────────────────
    def _parse_use_wiggle(self, text: str) -> bool:
        normalized = text.lower().strip().replace(' ', '')
        for kw in _NO_WIGGLE_KEYWORDS:
            if kw in normalized:
                return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = NlpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
