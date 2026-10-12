import sys
import time


def main():
    sys.stderr.write('[ERROR] [test_error_logger]: simulated error from launched process\n')
    sys.stderr.flush()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()