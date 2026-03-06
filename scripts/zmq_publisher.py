"""ZMQ Publisher for the voice control pipeline.

Run this in the language_control conda environment.

Usage:
  python zmq_publisher.py --mode text     # typed input (testing)
  python zmq_publisher.py --mode mic      # microphone input (live)
  python zmq_publisher.py --port 5556     # custom port

Publishes RobotCommand JSON on tcp://*:5556
"""

import argparse
import json
import time

import zmq


def main() -> None:
    parser = argparse.ArgumentParser(description="ZMQ Publisher for voice control")
    parser.add_argument("--mode", choices=["text", "mic"], default="text")
    parser.add_argument("--port", type=int, default=5556)
    parser.add_argument("--config", default="config/config.yaml")
    args = parser.parse_args()

    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://*:{args.port}")
    print(f"[ZMQ Publisher] Bound to tcp://*:{args.port}")
    time.sleep(0.5)  # slow joiner: let subscribers connect

    # Import voice pipeline (must be in language_control conda env)
    try:
        from pipeline.pipeline import VoiceControlPipeline

        pipeline = VoiceControlPipeline(args.config)
    except ImportError:
        print("[WARNING] Could not import VoiceControlPipeline.")
        print("[WARNING] Running in standalone test mode — type JSON manually.")
        pipeline = None

    print(f"[ZMQ Publisher] Mode: {args.mode}")
    print("[ZMQ Publisher] Ready. Publishing commands...")

    try:
        while True:
            if pipeline is None:
                raw = input("JSON> ")
                socket.send_string(raw)
                print(f"  Published: {raw}")
            elif args.mode == "text":
                text = input("Voice command> ")
                if text.lower() in ("quit", "exit"):
                    break
                result = pipeline.process_text(text)
                json_str = json.dumps(result)
                socket.send_string(json_str)
                print(f"  Published: {json_str}")
            elif args.mode == "mic":
                input("Press ENTER to record...")
                result = pipeline.process_microphone()
                json_str = json.dumps(result)
                socket.send_string(json_str)
                print(f"  Published: {json_str}")
    except KeyboardInterrupt:
        print("\n[ZMQ Publisher] Shutting down.")
    finally:
        socket.close()
        context.term()


if __name__ == "__main__":
    main()
