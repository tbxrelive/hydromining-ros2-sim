#!/usr/bin/env python3
"""Rebuild local procedural assets, including Ogre2 premultiplied-alpha sprites."""
from pathlib import Path
import numpy as np
from PIL import Image


def generate(destination):
    """Write deterministic textures and a unit +Z tapered water column."""
    textures = destination / 'materials/textures'
    meshes = destination / 'meshes'
    textures.mkdir(parents=True, exist_ok=True)
    meshes.mkdir(parents=True, exist_ok=True)
    y, x = np.mgrid[-1:1:64j, -1:1:64j]
    alpha = np.maximum(0, 1-x*x-y*y)**2 * .35
    srgb = np.where(alpha <= .0031308, 12.92*alpha,
                    1.055*alpha**(1/2.4)-.055)
    rgba = np.empty((64,64,4), dtype=np.uint8)
    rgba[:,:,:3] = np.round(srgb[:,:,None]*255).astype(np.uint8)
    rgba[:,:,3] = np.round(alpha*255).astype(np.uint8)
    rgba[rgba[:,:,3] == 0,:3] = 0  # alpha quantization must not leave additive borders
    Image.fromarray(rgba).save(textures / 'mist.png')
    fade = np.linspace(1, 0, 256)
    Image.fromarray(np.round(np.tile(fade[None,:,None], (1,1,4))*255).astype(np.uint8)).save(
        textures / 'fade.png')
    y, x = np.mgrid[0:128, 0:32]
    rgb = np.zeros((128,32,3), dtype=np.uint8)
    for i, base in enumerate([165,220,240]):
        rgb[:,:,i] = np.clip(base+22*np.sin(y*.32+x*.22), 0, 255)
    Image.fromarray(rgb).save(textures / 'water.png')
    lines = ['# Unit tapered jet, +Z; scale radius/radius/length at runtime.']
    for z in (0,1):
        for i in range(24):
            angle = i*2*np.pi/24
            radius = 1+z*.6
            lines.append(f'v {radius*np.cos(angle):.7f} {radius*np.sin(angle):.7f} {z}')
    for z in (0,1):
        for i in range(24):
            lines.append(f'vt {i/24:.7f} {z*4}')
    for i in range(24):
        j = (i+1)%24
        for face in ((i,j,j+24), (i,j+24,i+24)):
            lines.append('f '+' '.join(f'{n+1}/{n+1}' for n in face))
    (meshes / 'jet.obj').write_text('\n'.join(lines)+'\n')


if __name__ == '__main__':
    generate(Path(__file__).resolve().parent)
