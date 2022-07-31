#!/usr/bin/python3
import numpy as np
import subprocess
import click
from pathlib import Path
import time
from datetime import datetime
import os
import matplotlib.pyplot as plt
from shapely import geometry

@click.group('cli')
def cli():
    pass


@cli.command('run-single')
@click.option('-m', '--map', type=str, default='potholes')
@click.option('-n', '--number', type=int, default=1)
@click.option('-s', '--save', is_flag=True)
@click.option('-f', '--folder', type=str, default='')
def run_single(map, number, save, folder):
    date = datetime.utcnow()
    executable = "../build/EdgeVis_example"

    if not folder:
        now = Path('.') / str(date)
    else:
        now = Path(folder)
    now.mkdir(exist_ok=True)
    os.chdir(now)
    executable = "../" + executable
    arguments = ' -m ' + map + ' -n ' + str(number)
    if save:
        arguments += ' --save'
    command = executable + arguments
    out = subprocess.run(command, shell=True)

@cli.command('analyze')
@click.option('-e', '--edgevis', type=str, required=True)
@click.option('-p', '--polyvis', type=str, required=True)
def analyze(edgevis, polyvis):
    ePath = Path(edgevis)
    pPath = Path(polyvis)
    with ePath.open('r') as e:
        with pPath.open('r') as p:
            el = 1
            pl = 1
            while(True):
                try:
                    # --
                    el = e.readline()
                    pl = p.readline()
                    if not (el and pl):
                        break
                    # seeker
                    ePoint = e.readline()
                    pPoint = p.readline()
                    # number of polygon vertices
                    eCount = e.readline()
                    pCount = p.readline()
                    # actual polygon points
                    eVerts = e.readline().split(';')[:-1]
                    pVerts = p.readline().split(';')[:-1]
                    eVerts = np.array([eval(v) for v in eVerts])
                    pVerts = np.array([eval(v) for v in pVerts])
                    ePoly = geometry.Polygon([[p[0], p[1]] for p in eVerts])
                    pPoly = geometry.Polygon([[p[0], p[1]] for p in pVerts])

                    plt.figure()
                    plt.plot(eVerts[:, 0], eVerts[:, 1], linewidth=2, label=f'EdgeVis {ePoly.area}')
                    plt.plot(pVerts[:, 0], pVerts[:, 1], '--', label=f'PolyVis {pPoly.area}')
                    plt.legend()
                    plt.show()
                except NameError:
                    pass


if __name__ == "__main__":
    cli()