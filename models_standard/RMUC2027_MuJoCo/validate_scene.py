"""Portable MuJoCo checks using a generic test rover, not the user's robot.

Run: python validate_scene.py --render
All lengths are metres. Benchmark excludes rendering and deliberately uses
one dynamic rover in contact with the complete field at a 2 ms timestep.
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
from pathlib import Path
import platform
import statistics
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parent
COLLISION_GROUP=np.array([0,0,0,1,0,0],dtype=np.uint8)
ROUTES={
    'flat': {'start':[-11,-2], 'end':[-6.5,-2], 'speed':.6},
    'central_passage': {'start':[-3,-3.3], 'end':[-3,3.3], 'speed':.6},
    'ramp_red': {'start':[6,-2.4], 'end':[6,-5.2], 'speed':.45},
    'ramp_blue': {'start':[-6,2.4], 'end':[-6,5.2], 'speed':.45},
    'road_red': {'start':[5.7,6.3], 'end':[9.4,6.3], 'speed':.45},
    'road_blue': {'start':[-5.7,-6.3], 'end':[-9.4,-6.3], 'speed':.45},
}


def ray_height(m,d,x,y):
    gid=np.array([-1],dtype=np.int32);normal=np.zeros(3)
    dist=mujoco.mj_ray(m,d,np.array([x,y,5.]),np.array([0.,0.,-1.]),
                       COLLISION_GROUP,True,-1,gid,normal)
    if dist<0:raise AssertionError(f'No terrain at {x}, {y}')
    return 5-dist,int(gid[0]),normal


def virtual_model(root):
    assets={'field.xml':(ROOT/'field.xml').read_bytes()}
    assets.update({p.relative_to(ROOT).as_posix():p.read_bytes() for p in (ROOT/'assets').rglob('*.obj')})
    return mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'),assets=assets)


def prepare_rover(m,d,route):
    mujoco.mj_resetData(m,d)
    # Reset clears derived positions. Forward BEFORE any ray queries.
    mujoco.mj_forward(m,d)
    start=np.asarray(route['start'],float);delta=np.asarray(route['end'])-start
    yaw=math.atan2(delta[1],delta[0]);h,_,_=ray_height(m,d,*start)
    d.qpos[:3]=[*start,h+.25]
    d.qpos[3:7]=[math.cos(yaw/2),0,0,math.sin(yaw/2)]
    mujoco.mj_forward(m,d)
    return float(np.linalg.norm(delta))


def route_control(m,d,route):
    start=np.asarray(route['start'],float);end=np.asarray(route['end'],float)
    delta=end-start;length=np.linalg.norm(delta);direction=delta/length
    xy=d.qpos[:2];progress=float((xy-start)@direction)
    lateral=float(np.cross(np.r_[direction,0],np.r_[xy-start,0])[2])
    if d.time<.8 or progress>=length:
        d.ctrl[:]=0
    else:
        aim=start+direction*min(progress+.6,length+.2)
        wanted=math.atan2(aim[1]-xy[1],aim[0]-xy[0])
        body=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'test_rover')
        mat=d.xmat[body].reshape(3,3)
        actual=math.atan2(mat[1,0],mat[0,0])
        error=math.atan2(math.sin(wanted-actual),math.cos(wanted-actual))
        omega=float(np.clip(2.5*error,-.8,.8))
        speed=route['speed']*min(1.,(d.time-.8)/.5)
        left=(speed-.25*omega)/.1;right=(speed+.25*omega)/.1
        d.ctrl[:]=[left,right,left,right]
    return progress,lateral


def run_route(m,name,route):
    d=mujoco.MjData(m);length=prepare_rover(m,d,route)
    max_pen=0.;max_lateral=0.;min_up=1.;max_con=0;contact_steps=0;chassis_contacts=0
    path=[];snapshots={};done_time=None;post_settle_heights=[]
    body=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'test_rover')
    chassis=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'rover_chassis')
    max_time=length/route['speed']*2.5+3
    for i in range(int(max_time/m.opt.timestep)):
        progress,lateral=route_control(m,d,route)
        if progress>=length and done_time is None:done_time=d.time
        mujoco.mj_step(m,d)
        max_con=max(max_con,d.ncon);contact_steps+=d.ncon>0
        if d.ncon:
            max_pen=max(max_pen,max(0.,-float(np.min(d.contact.dist))))
            chassis_contacts+=int(any(chassis in c.geom for c in d.contact))
        if d.time>.8:
            max_lateral=max(max_lateral,abs(lateral));min_up=min(min_up,float(d.xmat[body].reshape(3,3)[2,2]))
            post_settle_heights.append(float(d.qpos[2]))
        if i%25==0:path.append([float(d.time),*d.qpos[:3].tolist(),int(d.ncon)])
        if 'road' in name and 'middle' not in snapshots and .45*length<progress<.65*length:
            snapshots['middle']=d.qpos.copy()
        if done_time is not None and d.time-done_time>.6:break
    progress,lateral=route_control(m,d,route)
    warnings=d.warning.number.astype(int).tolist()
    passed=(progress>=length-.08 and max_lateral<.30 and min_up>.5 and
            max_pen<.01 and sum(warnings)==0 and np.isfinite(d.qpos).all() and d.ncon>0)
    result={'name':name,'passed':bool(passed),'target_distance_m':length,'progress_m':progress,
            'simulation_s':float(d.time),'max_lateral_error_m':max_lateral,
            'max_contact_penetration_mm':max_pen*1000,'min_body_up_cosine':min_up,
            'max_contacts':int(max_con),'contact_step_fraction':contact_steps/(i+1),
            'chassis_contact_steps':chassis_contacts,'warnings':warnings,
            'final_position_m':d.qpos[:3].tolist(),
            'body_height_range_m':[min(post_settle_heights),max(post_settle_heights)]}
    return result,path,snapshots


def vertical_probes():
    locations=[('ground_red',10,2),('ground_blue',-10,-2),
               ('removed_facility_red',12.8,6.5),('removed_facility_blue',-12.8,-6.5),
               ('central_left',-3,-2),('central_right',3,2),
               ('trapezoid_red',6,-5),('trapezoid_blue',-6,5),
               ('road_flat_red',5.8,6.3),('road_flat_blue',-5.8,-6.3),
               ('step_lower_red',12,-2.9),('step_middle_red',12,-4.5),
               ('step_upper_red',12,-6.0),('step_lower_blue',-12,2.9),
               ('step_middle_blue',-12,4.5),('step_upper_blue',-12,6.0),
               ('seam_lower',12,-2.9426666666666668),('seam_underlying_ray',12,-2.955),
               ('seam_upper',12,-3.35),('seam_middle',12,-3.55)]
    base=mujoco.MjModel.from_xml_path(str(ROOT/'scene.xml'))
    dd=mujoco.MjData(base);mujoco.mj_forward(base,dd)
    xml=ET.parse(ROOT/'scene.xml').getroot()
    xml.find('compiler').set('discardvisual','true')
    wb=xml.find('worldbody');b=ET.SubElement(wb,'body',name='probe',pos='0 0 0')
    ET.SubElement(b,'joint',name='vertical_probe',type='slide',axis='0 0 1')
    ET.SubElement(b,'geom',name='probe_sphere',type='sphere',size='.035',mass='.5',
                  contype='2',conaffinity='1',solref='.006 1',friction='.9 .005 .0001')
    model=virtual_model(xml);data=mujoco.MjData(model)
    bid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'probe')
    results=[]
    for label,x,y in locations:
        h,_,normal=ray_height(base,dd,x,y)
        center_ray_height=h
        if label.startswith('seam_'):
            # A finite-radius sphere spans both sides of the shared edge.
            # Center rays at an exact float32 seam can hit a lower/underlying
            # face; use the upper adjacent interior surface as the reference.
            h,_,normal=max([ray_height(base,dd,x+dx,y) for dx in [-.0001,.0001]],key=lambda v:v[0])
        model.body_pos[bid]=[x,y,h+.20]
        mujoco.mj_resetData(model,data);mujoco.mj_forward(model,data)
        mujoco.mj_step(model,data,nstep=1250)
        expected=h+.035/abs(normal[2]);actual=model.body_pos[bid,2]+data.qpos[0]
        error=abs(actual-expected);warnings=data.warning.number.astype(int).tolist()
        results.append({'name':label,'position_xy_m':[x,y],'terrain_height_m':h,
                        'center_ray_height_m':center_ray_height,
                        'height_reference':'upper adjacent surface at x +/- 0.1 mm' if label.startswith('seam_') else 'center ray',
                        'expected_center_z_m':expected,'actual_center_z_m':float(actual),
                        'error_mm':float(error*1000),'contacts':int(data.ncon),'warnings':warnings,
                        'passed':bool(error<.0025 and data.ncon>0 and not sum(warnings))})
    return results


def obstacle_stop(m):
    d=mujoco.MjData(m)
    route={'start':[12,-1.8],'end':[12,-3.2],'speed':.25}
    prepare_rover(m,d,route)
    wall_frames=0;min_y=-1.8;max_pen=0.;first=None
    for _ in range(3000):
        route_control(m,d,route)
        if wall_frames>=100:d.ctrl[:]=0
        mujoco.mj_step(m,d)
        min_y=min(min_y,float(d.qpos[1]))
        if d.ncon:max_pen=max(max_pen,max(0.,-float(np.min(d.contact.dist))))
        hit=any(abs(c.frame.reshape(3,3)[0,2])<.4 for c in d.contact)
        if hit:
            wall_frames+=1
            if first is None:first=d.qpos[:3].tolist()
    passed=wall_frames>=100 and min_y>-2.55 and max_pen<.01 and not sum(d.warning.number)
    return {'name':'retained_step_blocks_rover','passed':bool(passed),
            'wall_contact_frames':wall_frames,'first_wall_contact_m':first,'minimum_y_m':min_y,
            'step_front_y_m':-2.6522,'max_contact_penetration_mm':max_pen*1000,
            'warnings':d.warning.number.astype(int).tolist()}


def cpu_name():
    try:
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,r'HARDWARE\DESCRIPTION\System\CentralProcessor\0') as key:
            return winreg.QueryValueEx(key,'ProcessorNameString')[0].strip()
    except (ImportError,OSError):return platform.processor()


def benchmark():
    root=ET.parse(ROOT/'rover_test.xml').getroot();root.find('compiler').set('discardvisual','true')
    m=virtual_model(root);d=mujoco.MjData(m);results=[]
    workloads=[('central_contact',[-3,-1.5],[-3,1],7.),
               ('road_bump_contact',[6.5,6.3],[8,6.3],4.5)]
    for label,start,end,speed in workloads:
        route={'start':start,'end':end,'speed':speed*.1}
        prepare_rover(m,d,route);mujoco.mj_step(m,d,nstep=800)
        d.ctrl[:]=speed
        state_spec=mujoco.mjtState.mjSTATE_INTEGRATION
        state=np.zeros(mujoco.mj_stateSize(m,state_spec));mujoco.mj_getState(m,d,state,state_spec)
        durations=[];contacts=[];warning=0
        for repeat in range(60):
            mujoco.mj_setState(m,d,state,state_spec);mujoco.mj_forward(m,d)
            elapsed=0.
            for chunk in range(5):
                t=time.perf_counter();mujoco.mj_step(m,d,nstep=100);elapsed+=time.perf_counter()-t
                contacts.append(int(d.ncon))
            warning+=int(sum(d.warning.number));durations.append(elapsed)
        total_sim=60*500*m.opt.timestep;total_wall=sum(durations)
        results.append({'name':label,'passed':warning==0 and max(contacts)>0,
                        'simulation_s':total_sim,'wall_s':total_wall,'real_time_factor':total_sim/total_wall,
                        'mean_step_us':total_wall/(60*500)*1e6,
                        'median_trial_real_time_factor':(500*m.opt.timestep)/statistics.median(durations),
                        'contacts_observed_min_max':[min(contacts),max(contacts)],'warning_count':warning})
    return {'dynamic_model':'one generic 20 kg four-wheel rover', 'rendering':False,
            'control':'constant wheel speed; identical settled states restored between 1 s trials',
            'timing':'mj_step only; state restore, forward, logging and compilation excluded',
            'timestep_s':float(m.opt.timestep),'collision_geoms':1365,'discardvisual':True,
            'workloads':results}


def render_previews(m,snapshots):
    from PIL import Image
    opt=mujoco.MjvOption();opt.geomgroup[:]=[1,1,1,0,0,0]
    d=mujoco.MjData(m);prepare_rover(m,d,ROUTES['central_passage'])
    mujoco.mj_step(m,d,nstep=600)
    with mujoco.Renderer(m,height=1080,width=1920) as renderer:
        camera=mujoco.MjvCamera();camera.lookat[:]=[0,0,.7];camera.distance=34
        camera.azimuth=90;camera.elevation=-62
        renderer.update_scene(d,camera,scene_option=opt)
        Image.fromarray(renderer.render()).save(ROOT/'mujoco_overview.png')
        camera.lookat[:]=[0,0,.7];camera.distance=33;camera.azimuth=90;camera.elevation=-89.9
        renderer.update_scene(d,camera,scene_option=opt)
        Image.fromarray(renderer.render()).save(ROOT/'mujoco_top.png')
        if 'middle' in snapshots:
            mujoco.mj_resetData(m,d);d.qpos[:]=snapshots['middle'];mujoco.mj_forward(m,d)
            camera.lookat[:]=[7.5,6.3,.4];camera.distance=5.3;camera.azimuth=115;camera.elevation=-35
            renderer.update_scene(d,camera,scene_option=opt)
            Image.fromarray(renderer.render()).save(ROOT/'mujoco_road_contact.png')
    return ['mujoco_overview.png','mujoco_top.png','mujoco_road_contact.png']


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--render',action='store_true')
    parser.add_argument('--report',type=Path,default=ROOT/'test_results.json')
    args=parser.parse_args()
    counts=[]
    for name in ['scene.xml','scene_physics.xml','rover_test.xml']:
        start=time.perf_counter();model=mujoco.MjModel.from_xml_path(str(ROOT/name))
        sec=time.perf_counter()-start;data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        ncollision=int(np.count_nonzero(model.geom_contype==1))
        assert ncollision==1365,(name,ncollision)
        if name=='scene_physics.xml':assert model.ngeom==1365 and model.nmesh==1352
        counts.append({'file':name,'compile_s':sec,'ngeom':model.ngeom,'nmesh':model.nmesh,
                       'nbody':model.nbody,'nq':model.nq,'collision_geoms':ncollision})
    m=mujoco.MjModel.from_xml_path(str(ROOT/'rover_test.xml'))
    routes=[];paths={};road_snapshot={}
    for name,route in ROUTES.items():
        result,path,snapshots=run_route(m,name,route)
        routes.append(result);paths[name]=path
        if name=='road_red':road_snapshot=snapshots
        print(json.dumps(result),flush=True)
    probes=vertical_probes();stop=obstacle_stop(m);perf=benchmark()
    images=render_previews(m,road_snapshot) if args.render else []
    passed=(all(r['passed'] for r in routes) and all(r['passed'] for r in probes) and
            stop['passed'] and all(r['passed'] for r in perf['workloads']))
    report={'status':'passed' if passed else 'failed','timestamp':datetime.datetime.now().astimezone().isoformat(),
            'environment':{'mujoco':mujoco.__version__,'python':platform.python_version(),
                           'platform':platform.platform(),'cpu':cpu_name()},
            'test_rover':{'mass_kg':20,'wheel_radius_m':.10,'wheel_width_m':.07,
                          'wheelbase_m':.46,'track_m':.50,'chassis_size_m':[.64,.4,.13],
                          'drive_torque_limit_per_wheel_Nm':6,'suspension':False,
                          'note':'generic test model; no user robot supplied'},
            'compiled_models':counts,'rover_routes':routes,'vertical_contact_probes':probes,
            'step_obstacle':stop,'benchmark':perf,'rendered_images':images,
            'limits':['Friction is an assumed tunable parameter, not a measured material property.',
                      'The retained high steps are obstacles for this rover, not claimed climbable.',
                      'CPU timing is one environment without rendering, not GPU training throughput.',
                      'These tests do not establish clearance or behaviour for a different robot.']}
    args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    (args.report.parent/'test_trajectories.json').write_text(json.dumps(paths),encoding='utf-8')
    print(json.dumps({'status':report['status'],'contact_probes_passed':sum(p['passed'] for p in probes),
                       'contact_probes_total':len(probes),'obstacle':stop,'benchmark':perf},indent=2),flush=True)
    if not passed:raise SystemExit(1)


if __name__=='__main__':main()
