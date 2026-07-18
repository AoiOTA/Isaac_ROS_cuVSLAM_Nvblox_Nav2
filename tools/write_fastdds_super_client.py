#!/usr/bin/env python3
"""Write a per-trial Fast DDS super-client profile for ROS introspection tools."""

from __future__ import annotations

import argparse
from pathlib import Path


TEMPLATE = """<?xml version="1.0" encoding="UTF-8" ?>
<dds>
  <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="jackal_super_client" is_default_profile="true">
      <rtps>
        <builtin>
          <discovery_config>
            <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
            <discoveryServersList>
              <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
                <metatrafficUnicastLocatorList>
                  <locator>
                    <udpv4>
                      <address>127.0.0.1</address>
                      <port>{port}</port>
                    </udpv4>
                  </locator>
                </metatrafficUnicastLocatorList>
              </RemoteServer>
            </discoveryServersList>
          </discovery_config>
        </builtin>
        <useBuiltinTransports>true</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        raise ValueError("Fast DDS discovery port must be in 1024..65535")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(TEMPLATE.format(port=args.port), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
