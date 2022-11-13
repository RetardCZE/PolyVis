#!/usr/bin/python3
import numpy as np
import subprocess
import click
from pathlib import Path
import time
from datetime import datetime
import os
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.patches import Polygon
from shapely import geometry
import pandas as pd
from matplotlib.backends.backend_pdf import PdfPages


@click.group('cli')
def cli():
    pass


@cli.command('show-results')
@click.option('-t', '--type', type= click.Choice(['triangular', 'polygonal']), default='triangular')
def show_results(type):
    from fpdf import FPDF
    from PIL import Image
    pdf = FPDF()

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
            try:
                image = "images/" + n + ".png"
                cover = Image.open(image)
                width, height = cover.size
                cover = cover.resize((int(width*0.2), int(height*0.2)))
                width, height = cover.size
                width, height = float(width * 0.264583), float(height * 0.264583)

                pdf_size ={'w': 210, 'h': 297}

                ratio = min(pdf_size['w']/width, pdf_size['h']/height)
                if ratio < 1:
                    ratio = 0.9 * ratio
                else:
                    ration = 1/ratio * 0.9

                pdf.add_page()
                #pdf.image(image, 10.5, 29, ratio * width, ratio * height)
                pdf.set_font('arial', 'B', 13.0)
                pdf.set_xy(75, 0)
                pdf.cell(ln=0, h=5.0, align='L', w=0, txt=f"Map:   {n}", border=0)
                pdf.set_font('arial', 'B', 13.0)
                pdf.set_xy(0, 9)
                pdf.cell(ln=0, h=5.0, align='L', w=0, txt=f"EdgeVis:   {e} s", border=0)
                pdf.set_xy(75, 9)
                pdf.cell(ln=0, h=5.0, align='L', w=0, txt=f"PolyVis:   {p} s", border=0)
                # pdf.set_xy(150, 9)
                # pdf.cell(ln=0, h=5.0, align='L', w=0, txt=f"TriVis:    {t} s", border=0)

                pdf.set_xy(0, 18)
                pdf.cell(ln=0, h=5.0, align='L', w=0, txt=f"           {100 * e/e:.2f}%", border=0)
                pdf.set_xy(75, 18)
                pdf.cell(ln=0, h=5.0, align='L', w=0, txt=f"           {100 * p/e:.2f}%", border=0)
                # pdf.set_xy(150, 18)
                # pdf.cell(ln=0, h=5.0, align='L', w=0, txt=f"           {100 * t/e:.2f}%", border=0)
            except FileNotFoundError as err:
                print(repr(err), image)
                pass

    pdf.output("report.pdf", "F")
    for key in times.keys():
        print(key, " - ", times[key])


@cli.command('run-all')
@click.option('-n', '--number', type=int, default=1)
@click.option('-t', '--type', type= click.Choice(['triangular', 'polygonal']), default='triangular')
def run_all(number, type):
    executable = "../build/EdgeVis_example"
    maps = Path("/home/jakub/Projects/IronHarvest/mesh-maps/iron-harvest")
    Path(f'results_{type}').mkdir(exist_ok=True)
    c = 0
    bad = []
    for m in maps.iterdir():
        if(m.suffix != '.mesh'):
            continue

        print(60*'-')
        c += 1
        name = m.stem
        print(name)
        arguments = ' -m ' + name + ' -n ' + str(number) + ' -t ' + type + ' --machine'
        command = executable + arguments
        out = subprocess.run(command, shell=True)
        try:
            os.replace("results.dat", f'results_{type}/{name}.dat')
        except FileNotFoundError:
            bad.append(name)
            print(f"{name} is not usable")

    print(bad)
    print(50*'-')

@cli.command('report-all')
@click.option('-t', '--type', type= click.Choice(['triangular', 'polygonal']), default='triangular')
def analyze(type):
    executable = "../build/EdgeVis_example"
    maps = Path("/home/jakub/Projects/IronHarvest/mesh-maps/iron-harvest")
    Path(f'results_{type}').mkdir(exist_ok=True)
    table = {}
    res = Path(f'results_{type}')
    df = pd.DataFrame()
    df2 = pd.DataFrame()
    headers = ['map',   # 0
               'PolyVis t',   #1
               'PolyVis exp',   #2
               'PolyVis depth',   #3
                  #
               'EdgeVis t',   #4
               'preprocessing edge',   #5
               'preprocessing v1',   #6
               'nodes v1',   #7
               'EdgeVis vertices',   #8
               'PolyVis / v1',   #9
                  #
               'EdgeVis2 t',   #10
               'preprocessing edge2',   #11
               'preprocessing v2',   #12
               'nodes v2',   #13
               'EdgeVis2 vertices',   #14
               'PolyVis / v2'   #15
               ]
    print(len(headers))
    df[headers[0]] = []
    # PolyVis
    df[headers[1]] = []
    df[headers[2]] = []
    df[headers[3]] = []
    # EdgeVis v1
    df[headers[4]] = []
    df[headers[5]] = []
    df[headers[6]] = []
    df[headers[7]] = []
    df[headers[8]] = []
    df[headers[9]] = []

    df[headers[10]] = []
    df[headers[11]] = []
    df[headers[12]] = []
    df[headers[13]] = []
    df[headers[14]] = []
    df[headers[15]] = []

    dfs = []
    lines = 0
    for f in res.iterdir():
        with f.open('r') as r:
            name = r.readline().replace('\n', '')
            if len(name) == 0:
                print(f.stem)
                continue
            r.readline()
            r.readline()
            df2[headers[0]] = [name]
            dataLine1 = (r.readline().replace('\n', '')).split(' ')
            df2[headers[4]] = [float(dataLine1[0])]
            df2[headers[5]] = [float(dataLine1[1])]
            df2[headers[6]] = [float(dataLine1[2])]
            df2[headers[7]] = [float(dataLine1[3])]
            df2[headers[8]] = [float(dataLine1[4])]

            dataLine3 = (r.readline().replace('\n', '')).split(' ')

            df2[headers[10]] = [float(dataLine3[0])]
            df2[headers[11]] = [float(dataLine3[1])]
            df2[headers[12]] = [float(dataLine3[2])]
            df2[headers[13]] = [float(dataLine3[3])]
            df2[headers[14]] = [float(dataLine3[4])]



            dataLine2 = (r.readline().replace('\n', '')).split(' ')
            # PolyVis
            df2[headers[1]] = [float(dataLine2[0])]
            df2[headers[2]] = [float(dataLine2[1])]
            df2[headers[3]] = [float(dataLine2[2])]

            df2[headers[15]] = [float(dataLine2[0]) / float(dataLine3[0])]
            df2[headers[9]] = [float(dataLine2[0]) / float(dataLine1[0])]
            df = df.append(df2, ignore_index=True)
        lines += 1
        if lines > 19:
            lines = 0
            dfs.append(df.copy())
            df = pd.DataFrame()
            df[headers[0]] = []
            # PolyVis
            df[headers[1]] = []
            df[headers[2]] = []
            df[headers[3]] = []
            # EdgeVis v1
            df[headers[4]] = []
            df[headers[5]] = []
            df[headers[6]] = []
            df[headers[7]] = []
            df[headers[8]] = []
            df[headers[9]] = []

            df[headers[10]] = []
            df[headers[11]] = []
            df[headers[12]] = []
            df[headers[13]] = []
            df[headers[14]] = []
            df[headers[15]] = []
    if lines > 0:
        dfs.append(df)

    for df in dfs:
        df.update(df[[headers[4],
                      headers[5],
                      headers[6],
                      headers[7],
                      headers[8],
                      headers[9],
                      headers[1],
                      headers[2],
                      headers[3]]].astype(float))
        df.update(df[[headers[4],
                      headers[5],
                      headers[6],
                      headers[7],
                      headers[8],
                      headers[9],
                      headers[1],
                      headers[2],
                      headers[3]]].applymap('{:,.3f}'.format))

    pdf = PdfPages('report_time.pdf')
    for df in dfs:
        fig = plt.figure()
        ax = fig.gca()
        ax.axis('off')

        dataFrameTimes = df[[headers[0], headers[4], headers[10], headers[1], headers[9], headers[15]]].copy()
        r, c = dataFrameTimes.shape
        # plot the real table
        table = ax.table(cellText=np.vstack([dataFrameTimes.columns, dataFrameTimes.values]),
                         cellColours=[['lightgray'] * c] * (r+1),
                         loc='upper center',
                         )
        table.auto_set_font_size(False)
        table.set_fontsize(6)
        table.auto_set_column_width(col=list(range(c)))  # Provide integer list of columns to adjust

        # need to draw here so the text positions are calculated
        fig.canvas.draw()
        pdf.savefig()
        plt.close()
    pdf.close()

    pdf = PdfPages('report_steps.pdf')
    for df in dfs:

        fig = plt.figure()
        ax = fig.gca()
        ax.axis('off')

        dataFrameTimes = df[[headers[0], headers[2], headers[3], headers[13], headers[14], headers[7], headers[8]]].copy()
        r, c = dataFrameTimes.shape
        # plot the real table
        table = ax.table(cellText=np.vstack([dataFrameTimes.columns, dataFrameTimes.values]),
                         cellColours=[['lightgray'] * c] * (r + 1),
                         loc='upper center',
                         )
        table.auto_set_font_size(False)
        table.set_fontsize(6)
        table.auto_set_column_width(col=list(range(c)))  # Provide integer list of columns to adjust

        # need to draw here so the text positions are calculated
        fig.canvas.draw()
        pdf.savefig()
        plt.close()
    pdf.close()

    pdf = PdfPages('report_preprocessing.pdf')
    for df in dfs:
        fig = plt.figure()
        ax = fig.gca()
        ax.axis('off')

        dataFrameTimes = df[[headers[0], headers[6], headers[5], headers[12]]].copy()
        r, c = dataFrameTimes.shape
        # plot the real table
        table = ax.table(cellText=np.vstack([dataFrameTimes.columns, dataFrameTimes.values]),
                         cellColours=[['lightgray'] * c] * (r + 1),
                         loc='upper center',
                         )
        table.auto_set_font_size(False)
        table.set_fontsize(6)
        table.auto_set_column_width(col=list(range(c)))  # Provide integer list of columns to adjust

        # need to draw here so the text positions are calculated
        fig.canvas.draw()
        pdf.savefig()
        plt.close()
    pdf.close()

@cli.command('run-single')
@click.option('-m', '--map', type=str, default='potholes')
@click.option('-n', '--number', type=int, default=1)
@click.option('-s', '--save', is_flag=True)
@click.option('-f', '--folder', type=str, default='')
def run_single(map, number, save, folder):
    date = datetime.utcnow()
    executable = "build/EdgeVis_example"

    if not folder:
        now = Path('.') / str(date)
    else:
        now = Path(folder)
    now.mkdir(exist_ok=True)
    os.chdir(now)
    executable = "../" + executable
    arguments = ' -m ' + map + ' -n ' + str(number) + ' -t triangular'
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
                print(res[0] + res[1] + res[2], end = '                  \r')
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
                    diff = abs(ePoly.area - pPoly.area)
                    relDiff = diff / max(ePoly.area, pPoly.area)
                    if relDiff < 1e-6:
                        res[0] += 1
                    else:
                        print(relDiff)
                        #plt.plot(*ePoly.exterior.xy, "-")
                        #plt.plot(*pPoly.exterior.xy, "--")
                        res[1] += 1

                except NameError:
                    print("nan")
                    res[2] += 1
                    pass
        print(res)


if __name__ == "__main__":
    cli()