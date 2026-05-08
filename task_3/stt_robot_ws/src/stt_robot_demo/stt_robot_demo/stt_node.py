#!/usr/bin/env python3
"""
stt_node.py — STT 기반 음성 명령 수신 노드  [산출물 a]

마이크 입력을 받아 Google Speech Recognition으로 텍스트 변환 후
/stt_result (std_msgs/String) 토픽으로 퍼블리시한다.

퍼블리시:
  /stt_result  (std_msgs/String)  — 인식된 원본 텍스트

ROS2 파라미터:
  language       (str,   기본 "ko-KR")  — 인식 언어
  energy_threshold (int, 기본 300)      — 마이크 감도 임계값
  pause_threshold  (float, 기본 0.8)    — 발화 종료 판단 침묵 시간 [s]
  phrase_timeout   (float, 기본 3.0)    — 단어 인식 최대 대기 시간 [s]
  device_index     (int,  기본 -1)      — 마이크 장치 인덱스 (-1: 기본 장치)
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import speech_recognition as sr
except ImportError:
    raise ImportError(
        "speech_recognition 패키지가 없습니다.\n"
        "pip install SpeechRecognition pyaudio"
    )


class SttNode(Node):

    def __init__(self):
        super().__init__('stt_node')

        # ── ROS2 파라미터 선언 ─────────────────────────────
        self.declare_parameter('language',        'ko-KR')
        self.declare_parameter('energy_threshold', 300)
        self.declare_parameter('pause_threshold',  0.8)
        self.declare_parameter('phrase_timeout',   3.0)
        self.declare_parameter('device_index',    -1)

        lang             = self.get_parameter('language').value
        energy_threshold = self.get_parameter('energy_threshold').value
        pause_threshold  = self.get_parameter('pause_threshold').value
        phrase_timeout   = self.get_parameter('phrase_timeout').value
        device_index     = self.get_parameter('device_index').value

        # ── 퍼블리셔 ──────────────────────────────────────
        self._pub = self.create_publisher(String, '/stt_result', 10)

        # ── Recognizer 설정 ───────────────────────────────
        self._recognizer = sr.Recognizer()
        self._recognizer.energy_threshold = energy_threshold
        self._recognizer.pause_threshold  = pause_threshold
        self._recognizer.phrase_threshold = 0.3
        self._recognizer.dynamic_energy_threshold = True

        self._lang         = lang
        self._phrase_timeout = phrase_timeout
        self._device_index = device_index if device_index >= 0 else None

        # ── 수신 스레드 시작 ──────────────────────────────
        threading.Thread(target=self._listen_loop, daemon=True).start()

        self.get_logger().info(
            f'STT 노드 준비 — 언어: {lang}, '
            f'에너지 임계값: {energy_threshold}, '
            f'마이크: {"기본 장치" if self._device_index is None else self._device_index}'
        )

    # ── 마이크 수신 루프 ──────────────────────────────────
    def _listen_loop(self):
        log = self.get_logger()
        log.info('마이크 청취 시작 ...')

        with sr.Microphone(device_index=self._device_index) as source:
            # 주변 소음 기준점 보정 (2초)
            log.info('주변 소음 보정 중 (2초) ...')
            self._recognizer.adjust_for_ambient_noise(source, duration=2.0)
            log.info(f'보정 완료  에너지 임계값 → {self._recognizer.energy_threshold:.0f}')

            while rclpy.ok():
                try:
                    log.info('발화 대기 중 ...')
                    audio = self._recognizer.listen(
                        source,
                        timeout=None,               # 발화 시작까지 무제한 대기
                        phrase_time_limit=self._phrase_timeout,
                    )
                except sr.WaitTimeoutError:
                    continue

                # ── 비동기 인식 (메인 루프 블로킹 방지) ──
                threading.Thread(
                    target=self._recognize,
                    args=(audio,),
                    daemon=True,
                ).start()

    def _recognize(self, audio):
        log = self.get_logger()
        try:
            text = self._recognizer.recognize_google(
                audio,
                language=self._lang,
            )
            log.info(f'[STT 인식] "{text}"')

            msg = String()
            msg.data = text
            self._pub.publish(msg)

        except sr.UnknownValueError:
            log.debug('인식 불가 (묵음 또는 잡음)')
        except sr.RequestError as e:
            log.error(f'Google STT API 오류: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SttNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
