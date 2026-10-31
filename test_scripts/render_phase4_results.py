#!/usr/bin/env python3
"""Plot actual Gazebo RGB-D captures and measured retreat; uses no GUI backend."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    directory=args.directory
    report=json.loads((directory/'summary.json').read_text())
    assert report['passed']
    names=['01_before','02_after_first','03_after_second']
    labels=['初始煤壁','第一次喷射后','第二次喷射后（累计）']
    data=[np.load(directory/(n+'.npz')) for n in names]
    points=[d['world'] for d in data]
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    fig,axes=plt.subplots(2,3,figsize=(13.5,8.2),layout='constrained')
    for i,(name,label) in enumerate(zip(names,labels)):
        axes[0,i].imshow(plt.imread(directory/(name+'_rgb.png')))
        axes[0,i].set_title(label,fontproperties=font,fontsize=13)
        axes[0,i].axis('off')
    d=points[0]
    # The fixture's actual TCP axis is world +X. Keep the actual baseline geometry
    # coordinates so the measured dent location can be inspected independently.
    base_valid=np.isfinite(d).all(1)
    for i,pts in enumerate(points):
        good=base_valid&np.isfinite(pts).all(1)&(np.abs(d[:,1])<.26)&(np.abs(d[:,2]-.7)<.26)&(d[:,0]>1.15)
        changes=(pts[good,0]-d[good,0])*1000
        im=axes[1,i].scatter(d[good,1],d[good,2]-.7,c=changes,s=9,
                             cmap='magma',vmin=0,vmax=100,rasterized=True)
        axes[1,i].set_aspect('equal')
        axes[1,i].set_xlim(-.26,.26);axes[1,i].set_ylim(-.26,.26)
        axes[1,i].set_xlabel('煤壁横向 y / m',fontproperties=font)
        axes[1,i].set_ylabel('相对命中高度 z−0.7 / m',fontproperties=font)
        axes[1,i].grid(alpha=.2)
        title='喷前基准（0 mm）' if i==0 else f'点云实测最大退缩 {np.max(changes):.1f} mm'
        axes[1,i].set_title(title,fontproperties=font,fontsize=11)
    bar=fig.colorbar(im,ax=axes[1,:],shrink=.85,pad=.02)
    bar.set_label('相同像素沿喷射轴的退缩 / mm',fontproperties=font)
    a,b=report['first'],report['second']
    title=(f'真实 Gazebo 无 GUI 渲染与 RGB-D 点云｜20 MPa，每次 12 s\n'
           f'首喷：深度 {a["max_depth_m"]*100:.2f} cm，体积 {a["volume_removed_m3"]*1e6:.1f} cm³；'
           f'两次累计：深度 {b["cumulative_depth_m"]*100:.2f} cm，体积 {b["cumulative_volume_m3"]*1e6:.1f} cm³')
    fig.suptitle(title,fontproperties=font,fontsize=14)
    fig.text(.5,-.025,'临时参数未标定；上排为相机实际渲染，下排为点云测量。网格最大深度与点云分位数不是同一指标。',
             ha='center',fontproperties=font,fontsize=10)
    target=directory/'comparison.png'
    fig.savefig(target,dpi=150,bbox_inches='tight')
    plt.close(fig)
    print(target)


if __name__=='__main__':
    main()
