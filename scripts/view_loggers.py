#!/usr/bin/python3
import numpy as np
import subprocess
import click
from pathlib import Path
import time
from datetime import datetime
import os
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from shapely import geometry

@click.group('cli')
def cli():
    pass


@cli.command('show-results')
@click.option('-t', '--type', type= click.Choice(['triangular', 'polygonal']), default='triangular')
def run_all(type):
    times = {}
    res = Path(f'results_{type}')
    for f in res.iterdir():
        with f.open('r') as r:
            try:
                n = r.readline().strip('\n')
                for i in range(5):
                    r.readline()
                e = float(r.readline().split(':')[-1])
                for i in range(4):
                    r.readline()
                p = float(r.readline().split(':')[-1])
                try:
                    for i in range(4):
                        r.readline()
                    t = float(r.readline().split(':')[-1])
                    times[n] = [e, p, t]
                except:
                    times[n] = [e, p]
            except:
                print("something went wrong on", n)

    for key in times.keys():
        print(key, " - ", times[key])


@cli.command('run-all')
@click.option('-n', '--number', type=int, default=1)
@click.option('-t', '--type', type= click.Choice(['triangular', 'polygonal']), default='triangular')
def run_all(number, type):
    executable = "../build/EdgeVis_example"
    maps = Path("../source/data/maps")
    Path(f'results_{type}').mkdir(exist_ok=True)
    c = 0
    for m in maps.iterdir():
        print(60*'-')
        c += 1
        name = m.stem
        arguments = ' -m ' + name + ' -n ' + str(number) + ' -t ' + type
        command = executable + arguments
        out = subprocess.run(command, shell=True)
        try:
            os.replace("results.dat", f'results_{type}/{name}.dat')
        except FileNotFoundError:
            print(f"{name} is not usable")

    times = {}
    res = Path(f'results_{type}')
    for f in res.iterdir():
        with f.open('r') as r:
            try:
                n = r.readline().strip('\n')
                for i in range(5):
                    r.readline()
                e = float(r.readline().split(':')[-1])
                for i in range(4):
                    r.readline()
                p = float(r.readline().split(':')[-1])
                try:
                    for i in range(4):
                        r.readline()
                    t = float(r.readline().split(':')[-1])
                    times[n] = [e, p, t]
                except:
                    times[n] = [e, p]
            except:
                print("something went wrong on", n)

    for key in times.keys():
        print(key, " - ", times[key])



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
@click.option('-t', '--trivis', is_flag=True)
def analyze(edgevis, polyvis, trivis):
    ePath = Path(edgevis)
    pPath = Path(polyvis)
    res = [0,0,0]
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
                    if trivis:
                        pVerts = ((p.readline().replace("{", "(")).replace("}", ")")).split(';')[:-1]
                    else:
                        pVerts = p.readline().split(';')[:-1]
                    eVerts = np.array([eval(v) for v in eVerts])
                    pVerts = np.array([eval(v) for v in pVerts])
                    ePoly = geometry.Polygon([[p[0], p[1]] for p in eVerts])
                    pPoly = geometry.Polygon([[p[0], p[1]] for p in pVerts])
                    if abs(ePoly.area - pPoly.area) < 1e-8:
                        res[0] += 1
                    else:
                        print(ePoly.area - pPoly.area)
                        plt.plot(*ePoly.exterior.xy, "-")
                        plt.plot(*pPoly.exterior.xy, "--")
                        plt.show()
                        res[1] += 1

                except NameError:
                    print("nan")
                    res[2] += 1
                    pass
        print(res)


if __name__ == "__main__":
    cli()