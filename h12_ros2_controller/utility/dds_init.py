"""Helper to initialize the Unitree DDS ChannelFactory from ROS_DOMAIN_ID.

Raises ValueError if the env var isn't set so misconfigured launches fail
loudly instead of silently defaulting to domain 0.
"""
import os

from unitree_sdk2py.core.channel import ChannelFactoryInitialize


def init_channel_factory_from_env() -> None:
    domain_id = os.environ.get('ROS_DOMAIN_ID')
    if domain_id is None:
        raise ValueError(
            "ROS_DOMAIN_ID environment variable is not set; "
            "set it (e.g. `export ROS_DOMAIN_ID=1`) before launching."
        )
    ChannelFactoryInitialize(int(domain_id))
