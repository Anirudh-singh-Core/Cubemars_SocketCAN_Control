"""Offline shell workflow checks. All hardware commands and sysfs are fixtures."""
import json
import os
from pathlib import Path
import subprocess
import sys
import pytest

SOURCE=Path(__file__).resolve().parents[1]/'tools'/'setup_socketcan.sh'

@pytest.fixture
def setup_runner(tmp_path):
    root=tmp_path/'sys';board=tmp_path/'model';board.write_text('NVIDIA Jetson Orin Nano\0')
    driver=root/'bus/platform/drivers/mttcan';driver.mkdir(parents=True)
    device=root/'devices/c310000.mttcan';(device/'net/can0').mkdir(parents=True)
    (driver/'c310000.mttcan').symlink_to(device)
    (device/'driver').symlink_to(driver)
    (root/'class/net').mkdir(parents=True)
    (root/'class/net/can0').symlink_to(device/'net/can0')
    (device/'net/can0/device').symlink_to(device)
    (root/'module/mttcan').mkdir(parents=True)
    (root/'module/mttcan/refcnt').write_text('0')
    home=tmp_path/'home';(home/'.cache/handyflie').mkdir(parents=True)
    # Only the test copy bypasses root and reads fixture sysfs. Production has no bypass.
    script=tmp_path/'setup.sh'
    script.write_text(SOURCE.read_text().replace('$EUID','0')
        .replace('/proc/device-tree/model',str(board)).replace('/sys/',str(root)+'/'))
    commands=tmp_path/'bin';commands.mkdir()
    stub=commands/'stub'
    stub.write_text('#!'+sys.executable+'''\nimport json,os,sys
from pathlib import Path
name=Path(sys.argv[0]).name;a=sys.argv[1:]
with open(os.environ['TRACE'],'a') as f:f.write(json.dumps([name,*a])+'\\n')
if name=='getent':print('fixture:x:1000:1000:fixture:'+os.environ['TEST_HOME']+':/bin/bash')
elif name=='id':print('fixture')
elif name=='install':pass
elif name=='flock':sys.exit(1 if os.environ.get('OWNED') else 0)
elif name=='busybox':print('0x0000c400' if '0x0c303010' in a else '0x0000c458')
elif name=='modprobe':
 if '-r' in a:
  if os.environ.get('RELOAD_FAIL'):sys.exit(1)
  Path(os.environ['RECOVERED']).touch()
elif name=='ip':
 if '-json' in a:
  ready=Path(os.environ['CONFIGURED']).exists()
  state='ERROR-ACTIVE' if ready else ('STOPPED' if Path(os.environ['RECOVERED']).exists() else 'ERROR-PASSIVE')
  if ready and os.environ.get('VERIFY_FAIL'):state='ERROR-WARNING'
  print(json.dumps([dict(flags=['UP'] if ready else [],linkinfo=dict(info_kind='can',info_data=dict(
   state=state,bittiming=dict(bitrate=1000000),restart_ms=0,ctrlmode=['BERR-REPORTING'],
   berr_counter=dict(tx=0,rx=0))))]))
 elif a[-1:] == ['up']:Path(os.environ['CONFIGURED']).touch()
else:sys.exit('Unexpected mock command '+name)
''')
    stub.chmod(0o755)
    for name in ('getent','id','install','flock','busybox','modprobe','ip'):(commands/name).symlink_to(stub)
    trace=tmp_path/'trace'
    def run(*,recover=True,extra_env=None,other=False):
        if other:
            other_device=root/'devices/c320000.mttcan';other_device.mkdir()
            (driver/other_device.name).symlink_to(other_device)
        env=dict(os.environ,PATH=str(commands)+':'+os.environ['PATH'],SUDO_USER='fixture',
                 TEST_HOME=str(home),TRACE=str(trace),RECOVERED=str(tmp_path/'recovered'),
                 CONFIGURED=str(tmp_path/'configured'))
        env.update(extra_env or {})
        args=['bash',str(script),'--confirm-stopped','--jetson-orin-nano']
        if recover:args.append('--recover-driver')
        result=subprocess.run(args,env=env,text=True,capture_output=True,timeout=5)
        calls=[json.loads(x) for x in trace.read_text().splitlines()] if trace.exists() else []
        return result,calls
    return run


def mutations(calls):
    return [c for c in calls if c[:2]==['modprobe','-r'] or c[:3]==['ip','link','set']]


def test_explicit_recovery_reloads_once_and_verifies(setup_runner):
    r,calls=setup_runner()
    assert r.returncode==0,r.stderr
    assert calls.count(['modprobe','-r','mttcan'])==1
    down=next(i for i,c in enumerate(calls) if c[:3]==['ip','link','set'] and c[-1]=='down')
    assert down<calls.index(['modprobe','-r','mttcan'])<calls.index(['modprobe','mttcan'])
    config=next(c for c in calls if c[:3]==['ip','link','set'] and 'bitrate' in c)
    assert '1000000' in config and config[-2:]==['restart-ms','0']
    assert 'CAN ready at 1 Mbps' in r.stdout


@pytest.mark.parametrize('case',['owned','other','reload_fail','verify_fail','normal_setup'])
def test_refusal_and_failure_paths(setup_runner,case):
    env={'owned':{'OWNED':'1'},'reload_fail':{'RELOAD_FAIL':'1'},'verify_fail':{'VERIFY_FAIL':'1'}}.get(case,{})
    r,calls=setup_runner(extra_env=env,other=case=='other',recover=case!='normal_setup')
    assert r.returncode!=0
    expected={'owned':'channel is owned','other':'another controller','reload_fail':'Recovery failed',
              'verify_fail':'verification failed','normal_setup':'CAN is unhealthy'}[case]
    assert expected in r.stdout+r.stderr,(r.stdout,r.stderr)
    assert 'CAN ready at 1 Mbps' not in r.stdout
    if case in ('owned','other','normal_setup'):assert mutations(calls)==[]
    if case=='reload_fail':assert not any(c[-1]=='up' for c in mutations(calls))


def test_recovery_requires_explicit_jetson_flag():
    r=subprocess.run(['bash',str(SOURCE),'--confirm-stopped','--recover-driver'],capture_output=True,text=True)
    assert r.returncode==2 and 'requires --jetson-orin-nano' in r.stderr
