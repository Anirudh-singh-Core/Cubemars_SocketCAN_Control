"""Read-only Linux interface inspection. Setup is an explicit administrator action."""
import json
import re
import subprocess
from .errors import InterfaceError

FAULT_COUNTERS=('rx_errors','tx_errors','rx_dropped','tx_dropped',
                'bus_error','bus_off','error_warning','error_passive','restarts')

def validate_channel(channel):
    if not isinstance(channel,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,15}',channel):
        raise ValueError('Invalid CAN interface name')
    return channel

def inspect_interface(channel='can0'):
    validate_channel(channel)
    try:
        output=subprocess.check_output(['ip','-details','-statistics','-json','link',
                                        'show','dev',channel],text=True,timeout=1)
        info=json.loads(output)[0]
    except (OSError,subprocess.SubprocessError,ValueError,IndexError) as exc:
        raise InterfaceError(f'Cannot inspect {channel}: {exc}') from exc
    link=info.get('linkinfo',{});data=link.get('info_data',{})
    stats=info.get('stats64',info.get('stats',{}))
    counters={f'{direction}_{key}':stats.get(direction,{}).get(key,0)
              for direction in ('rx','tx') for key in ('packets','errors','dropped')}
    counters.update(link.get('info_xstats',{}))
    reasons=[]
    if link.get('info_kind')!='can':reasons.append('Not a physical CAN interface')
    if 'UP' not in info.get('flags',[]):reasons.append('Interface is down')
    if data.get('bittiming',{}).get('bitrate')!=1000000:reasons.append('Require 1 Mbps')
    if data.get('state')!='ERROR-ACTIVE':reasons.append(f"Controller state: {data.get('state')}")
    if {'LOOPBACK','LISTEN-ONLY','FD'} & set(data.get('ctrlmode',[])):
        reasons.append('Require normal classic CAN mode')
    return dict(channel=channel,ready=not reasons,reasons=reasons,
                state=data.get('state'),counters=counters,details=info)

def require_ready(channel):
    status=inspect_interface(channel)
    if not status['ready']:raise InterfaceError('; '.join(status['reasons']))
    return status
