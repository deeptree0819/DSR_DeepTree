#!/usr/bin/env python3
"""
tts_node.py — TTS 기반 작업 상태 안내 노드  [산출물 d]

/tts_input (String) 구독 → gTTS(Google TTS)로 음성 합성 → pygame으로 재생
/speak 서비스 제공 → 다른 노드가 직접 TTS 요청 가능

구독:
  /tts_input  (std_msgs/String)   — 자동 음성 출력

서비스:
  /speak      (stt_robot_interfaces/Speak)  — 수동 음성 출력 요청

ROS2 파라미터:
  language    (str,   기본 "ko")    — TTS 언어 코드
  rate        (float, 기본 1.0)     — 발화 속도 (0.5 ~ 2.0)
  volume      (float, 기본 0.9)     — 출력 볼륨 (0.0 ~ 1.0)
  slow        (bool,  기본 false)   — gTTS slow 모드
"""

import io
import os
import queue
import tempfile
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from stt_robot_interfaces.srv import Speak

try:
    from gtts import gTTS
    _GTTS_OK = True
except ImportError:
    _GTTS_OK = False

try:
    import pygame
    _PYGAME_OK = True
except ImportError:
    _PYGAME_OK = False


class TtsNode(Node):

    def __init__(self):
        super().__init__('tts_node')

        # ── 파라미터 ──────────────────────────────────────
        self.declare_parameter('language', 'ko')
        self.declare_parameter('rate',     1.0)
        self.declare_parameter('volume',   0.9)
        self.declare_parameter('slow',     False)

        self._lang   = self.get_parameter('language').value
        self._rate   = self.get_parameter('rate').value
        self._volume = self.get_parameter('volume').value
        self._slow   = self.get_parameter('slow').value

        # ── pygame 초기화 ─────────────────────────────────
        if _PYGAME_OK:
            pygame.mixer.init()
            pygame.mixer.music.set_volume(self._volume)
        else:
            self.get_logger().warn(
                'pygame 없음 — 음성 출력 불가. pip install pygame'
            )

        if not _GTTS_OK:
            self.get_logger().warn(
                'gTTS 없음 — 음성 합성 불가. pip install gtts'
            )

        # ── 재생 큐 + 워커 스레드 ─────────────────────────
        self._tts_q: queue.Queue[tuple[str, float]] = queue.Queue()
        threading.Thread(target=self._tts_worker, daemon=True).start()

        # ── 구독 ──────────────────────────────────────────
        self.create_subscription(
            String, '/tts_input', self._tts_input_cb, 10
        )

        # ── 서비스 서버 ───────────────────────────────────
        self.create_service(Speak, '/speak', self._handle_speak)

        self.get_logger().info(
            f'TTS 노드 준비 완료 — 언어: {self._lang}, '
            f'속도: {self._rate}, 볼륨: {self._volume}'
        )

    # ── 토픽 콜백 ─────────────────────────────────────────
    def _tts_input_cb(self, msg: String):
        if msg.data:
            self.get_logger().info(f'[TTS 수신] "{msg.data}"')
            self._tts_q.put((msg.data, self._rate))

    # ── 서비스 콜백 ───────────────────────────────────────
    def _handle_speak(self, request, response):
        text     = request.text
        rate     = request.rate if request.rate > 0.0 else self._rate
        blocking = request.blocking

        if not text:
            response.success = False
            response.message = '텍스트가 비어있습니다.'
            return response

        self.get_logger().info(f'[/speak 서비스] "{text}" (blocking={blocking})')

        if blocking:
            self._speak(text, rate)
        else:
            self._tts_q.put((text, rate))

        response.success = True
        response.message = 'OK'
        return response

    # ── TTS 워커 (직렬 재생) ──────────────────────────────
    def _tts_worker(self):
        while True:
            try:
                text, rate = self._tts_q.get(timeout=1.0)
            except queue.Empty:
                continue
            self._speak(text, rate)

    def _speak(self, text: str, rate: float = 1.0):
        """
        [산출물 d] gTTS로 텍스트를 음성 합성한 뒤 pygame으로 재생.
        gTTS / pygame 미설치 시 로그로 대체 출력.
        """
        log = self.get_logger()

        if not _GTTS_OK or not _PYGAME_OK:
            log.info(f'[TTS 출력(텍스트)] {text}')
            return

        try:
            tts = gTTS(text=text, lang=self._lang, slow=self._slow)

            with tempfile.NamedTemporaryFile(
                suffix='.mp3', delete=False
            ) as tmp:
                tmp_path = tmp.name
                tts.save(tmp_path)

            pygame.mixer.music.load(tmp_path)
            pygame.mixer.music.set_volume(self._volume)
            pygame.mixer.music.play()

            # 재생 완료까지 대기
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)

            os.unlink(tmp_path)
            log.info(f'[TTS 재생 완료] "{text}"')

        except Exception as e:
            log.error(f'TTS 재생 오류: {e}')
            log.info(f'[TTS 출력(텍스트)] {text}')


def main(args=None):
    rclpy.init(args=args)
    node = TtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
