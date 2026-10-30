#!/usr/bin/env python3
"""Installed ROS interface regression, no Gazebo or GUI required."""
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_srvs.srv import Trigger
from hydromining_robot.srv import SprayAction
from ament_index_python.packages import get_package_prefix


def main():
    """Cover startup validation, lifecycle, live parameters and transactional replay."""
    out=Path(tempfile.mkdtemp(prefix='hydromining_ros_test_'))
    executable=Path(get_package_prefix('hydromining_robot'))/'lib/hydromining_robot/spray_controller_node.py'
    command=[sys.executable,str(executable),'--ros-args','-p','update_gazebo:=false',
             '-p','output_mesh_dir:='+str(out/'meshes')]
    invalid=subprocess.run(command+['-p','dt:=-1.0'],cwd='/tmp',capture_output=True,text=True,timeout=20)
    assert invalid.returncode!=0 and 'dt must be' in invalid.stderr,invalid.stderr
    log=(out/'node.log').open('w')
    process=subprocess.Popen(command,cwd='/tmp',stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();node=Node('spray_interface_regression')

    def call(kind,name,request):
        client=node.create_client(kind,name)
        assert client.wait_for_service(timeout_sec=20),name
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=40)
        assert future.done(),name
        result=future.result();node.destroy_client(client)
        return result

    try:
        deadline=time.monotonic()+25
        while time.monotonic()<deadline:
            state=call(GetState,'/spray_controller_node/get_state',GetState.Request())
            if state.current_state.id==3:
                break
            time.sleep(.1)
        assert state.current_state.id==3
        dt=Parameter(name='dt',value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=-1.))
        result=call(SetParameters,'/spray_controller_node/set_parameters',SetParameters.Request(parameters=[dt]))
        assert not result.results[0].successful
        request=SprayAction.Request(use_tcp=False,position_z=1.,direction_z=-1.,pressure=20.,duration=12.,enabled=True)
        first=call(SprayAction,'/spray_action',request)
        assert first.success and first.max_depth_m>0 and first.volume_removed_m3>0,first.message
        bad=SprayAction.Request(position_z=1.,direction_z=-2.,pressure=20.,duration=1.,enabled=True)
        assert not call(SprayAction,'/spray_action',bad).success
        changed=call(ChangeState,'/spray_controller_node/change_state',
                     ChangeState.Request(transition=Transition(id=Transition.TRANSITION_DEACTIVATE)))
        assert changed.success
        assert not call(SprayAction,'/spray_action',request).success
        changed=call(ChangeState,'/spray_controller_node/change_state',
                     ChangeState.Request(transition=Transition(id=Transition.TRANSITION_ACTIVATE)))
        assert changed.success
        # Reactivation exports the retained state, not an unrelated initial mesh.
        assert (out/'meshes/coal_wall_0003.obj').read_bytes()==Path(first.output_mesh_path).read_bytes()
        reset=call(Trigger,'/reset_coal_wall',Trigger.Request());assert reset.success
        replay=call(SprayAction,'/spray_action',request);assert replay.success
        assert Path(first.output_mesh_path).read_bytes()==Path(replay.output_mesh_path).read_bytes()
        print(json.dumps({'passed':True,'working_directory':'/tmp','lifecycle_active':True,
              'invalid_startup_dt_rejected':True,'negative_runtime_dt_rejected':True,
              'invalid_direction_rejected':True,'inactive_request_rejected':True,
              'reactivation_preserves_geometry':True,'reset_replay_mesh_identical':True,
              'volume_removed_m3':first.volume_removed_m3,'max_depth_m':first.max_depth_m,
              'logs':str(out)},indent=2))
        return 0
    finally:
        node.destroy_node();rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            process.terminate();process.wait(timeout=5)
        log.close()


if __name__=='__main__':
    sys.exit(main())
