""" protocol.py

    Author:     Mike Pagel <https://github.com/moltob>
    Date:       11/01/2016
    Modified:
        03/18/2019 by Carson Clarke-Magrab <ctc7359@rit.edu>
        -   Altered for IDE virtual NXP Cup.
"""
import argparse
import logging

from client.protocol import Client


def main():
    """Main entry point of application."""
    parser = argparse.ArgumentParser(description='Client for TORCS racing car simulation')
    parser.add_argument('--hostname', help='Server host name.', default='localhost')
    parser.add_argument('--port', help='Port to connect, 3001 - 3010 for clients 1 - 10.', type=int, default=3001)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)7s %(name)s %(message)s")

    client = Client(args.hostname, args.port)
    client.run()


if __name__ == '__main__':
    main()
