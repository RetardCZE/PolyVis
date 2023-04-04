#!/usr/bin/python3
import numpy as np
import subprocess
import click
from pathlib import Path
import os
from shapely import geometry
import pickle


@click.group('cli')
def cli():
    pass


@cli.command('run-all')
@click.option('-n', '--number', type=int, default=1)
@click.option('-r', '--robust', is_flag=True)
def run_all(number, robust):
    executable = f"../build/source/EdgeVis_example"
    maps = Path("../dependencies/IronHarvest/mesh-maps/iron-harvest")
    if robust:
        saveDir = Path(f'robust_times')
    else:
        saveDir = Path(f'nonrobust_times')

    saveDir.mkdir(exist_ok=True)
    c = 0
    bad = []
    for m in maps.iterdir():
        if(m.suffix != '.mesh'):
            continue

        print(60*'-')
        c += 1
        name = m.stem
        print(name)
        arguments = f' -m {name} -n {str(number)} --random-seed 1'
        if robust:
            command = executable + arguments + ' -r'
        else:
            command = executable + arguments
        subprocess.run(command, shell=True)
        try:
            os.replace("results.dat", f'{str(saveDir.resolve())}/{name}.dat')
        except FileNotFoundError:
            bad.append(name)
            print(f"{name} is not usable")

    print("List of maps for which test failed: ", bad)
    print(50*'-')


@cli.command('load')
def load():
    timeRobustFolder = Path('./robust_times')
    timeFastFolder = Path('./nonrobust_times')
    detailsRobustFolder = Path('./robust_details')
    detailsFastFolder =Path('./nonrobust_details')
    maps = {}
    for m in timeRobustFolder.iterdir():
        try:
            timeRobustFile = (timeRobustFolder / f'{m.stem}.dat').open('r')
            timeFastFile = (timeFastFolder / f'{m.stem}.dat').open('r')
            detailsRobustMapFolder = (detailsRobustFolder / f'{m.stem}')
            detailsFastMapFolder = (detailsFastFolder / f'{m.stem}')
            detailsRobustSummary = (detailsRobustMapFolder / 'sideResults.dat').open('r')
            detailsFastSummary = (detailsFastMapFolder / 'sideResults.dat').open('r')

            robustLoggers = [
                            (detailsRobustMapFolder / f'PEA_triangl.log').open('r'),
                            (detailsRobustMapFolder / f'PEA_polygon.log').open('r'),
                            (detailsRobustMapFolder / f'E1.log').open('r'),
                            (detailsRobustMapFolder / f'E2.log').open('r'),
                            (detailsRobustMapFolder / f'E3.log').open('r')
                            ]
            fastLoggers = [
                            (detailsFastMapFolder / f'PEA_triangl.log').open('r'),
                            (detailsFastMapFolder / f'PEA_polygon.log').open('r'),
                            (detailsFastMapFolder / f'E1.log').open('r'),
                            (detailsFastMapFolder / f'E2.log').open('r'),
                            (detailsFastMapFolder / f'E3.log').open('r')
                            ]
        except FileNotFoundError as err:
            print("[Warning] Please run both robust and non robust test before trying to evaluate results. See docu.")
        maps[m.stem] = {'robust': {},
                          'fast': {}}

        # name
        timeRobustFile.readline().replace('\n', '')
        timeFastFile.readline().replace('\n', '')

        # type
        timeRobustFile.readline().replace('\n', '')
        timeFastFile.readline().replace('\n', '')

        # number of queries
        timeRobustFile.readline().replace('\n', '')
        timeFastFile.readline().replace('\n', '')

        # pre-processing of edges
        maps[m.stem]['robust']['searchnodes'] = float(timeRobustFile.readline().replace('\n', ''))
        maps[m.stem]['fast']['searchnodes'] = float(timeFastFile.readline().replace('\n', ''))

        # pre-processing EdgeVis1
        maps[m.stem]['robust']['optimnodes1'] = float(timeRobustFile.readline().replace('\n', ''))
        maps[m.stem]['fast']['optimnodes1'] = float(timeFastFile.readline().replace('\n', ''))

        # pre-processing EdgeVis2
        maps[m.stem]['robust']['optimnodes2'] = float(timeRobustFile.readline().replace('\n', ''))
        maps[m.stem]['fast']['optimnodes2'] = float(timeFastFile.readline().replace('\n', ''))

        # pre-processing EdgeVis3
        maps[m.stem]['robust']['optimnodes3'] = float(timeRobustFile.readline().replace('\n', ''))
        maps[m.stem]['fast']['optimnodes3'] = float(timeFastFile.readline().replace('\n', ''))

        # M-CDT
        maps[m.stem]['robust']['M-CDT'] = float(timeRobustFile.readline().replace('\n', ''))
        maps[m.stem]['fast']['M-CDT'] = float(timeFastFile.readline().replace('\n', ''))

        # CDT
        maps[m.stem]['robust']['CDT'] = float(timeRobustFile.readline().replace('\n', ''))
        maps[m.stem]['fast']['CDT'] = float(timeFastFile.readline().replace('\n', ''))

        # M-CDT grid
        maps[m.stem]['robust']['M-CDTgrid'] = float(timeRobustFile.readline().replace('\n', ''))
        maps[m.stem]['fast']['M-CDTgrid'] = float(timeFastFile.readline().replace('\n', ''))

        # CDT grid
        maps[m.stem]['robust']['CDT-grid'] = float(timeRobustFile.readline().replace('\n', ''))
        maps[m.stem]['fast']['CDT-grid'] = float(timeFastFile.readline().replace('\n', ''))

        maps[m.stem]['robust']['EdgeVis1'] = {}
        maps[m.stem]['fast']['EdgeVis1'] = {}
        maps[m.stem]['robust']['EdgeVis2'] = {}
        maps[m.stem]['fast']['EdgeVis2'] = {}
        maps[m.stem]['robust']['EdgeVis3'] = {}
        maps[m.stem]['fast']['EdgeVis3'] = {}
        maps[m.stem]['robust']['TEA'] = {}
        maps[m.stem]['fast']['TEA'] = {}
        maps[m.stem]['robust']['PEA'] = {}
        maps[m.stem]['fast']['PEA'] = {}


        # EdgeVis 1 [time, evaluated, final]
        dataR = (timeRobustFile.readline().replace('\n', '')).split(' ')
        dataF = (timeFastFile.readline().replace('\n', '')).split(' ')
        maps[m.stem]['robust']['EdgeVis1']['time'] = float(dataR[0])
        maps[m.stem]['fast']['EdgeVis1']['time'] = float(dataF[0])
        maps[m.stem]['robust']['EdgeVis1']['evald'] = float(dataR[1])
        maps[m.stem]['fast']['EdgeVis1']['evald'] = float(dataF[1])
        maps[m.stem]['robust']['EdgeVis1']['final'] = float(dataR[2])
        maps[m.stem]['fast']['EdgeVis1']['final'] = float(dataF[2])

        # EdgeVis 2 [time, evaluated, final]
        dataR = (timeRobustFile.readline().replace('\n', '')).split(' ')
        dataF = (timeFastFile.readline().replace('\n', '')).split(' ')
        maps[m.stem]['robust']['EdgeVis2']['time'] = float(dataR[0])
        maps[m.stem]['fast']['EdgeVis2']['time'] = float(dataF[0])
        maps[m.stem]['robust']['EdgeVis2']['evald'] = float(dataR[1])
        maps[m.stem]['fast']['EdgeVis2']['evald'] = float(dataF[1])
        maps[m.stem]['robust']['EdgeVis2']['final'] = float(dataR[2])
        maps[m.stem]['fast']['EdgeVis2']['final'] = float(dataF[2])

        # EdgeVis 3 [time, evaluated, final]
        dataR = (timeRobustFile.readline().replace('\n', '')).split(' ')
        dataF = (timeFastFile.readline().replace('\n', '')).split(' ')
        maps[m.stem]['robust']['EdgeVis3']['time'] = float(dataR[0])
        maps[m.stem]['fast']['EdgeVis3']['time'] = float(dataF[0])
        maps[m.stem]['robust']['EdgeVis3']['evald'] = float(dataR[1])
        maps[m.stem]['fast']['EdgeVis3']['evald'] = float(dataF[1])
        maps[m.stem]['robust']['EdgeVis3']['final'] = float(dataR[2])
        maps[m.stem]['fast']['EdgeVis3']['final'] = float(dataF[2])

        # TEA [time, avg exp, max depth]
        dataR = (timeRobustFile.readline().replace('\n', '')).split(' ')
        dataF = (timeFastFile.readline().replace('\n', '')).split(' ')
        maps[m.stem]['robust']['TEA']['time'] = float(dataR[0])
        maps[m.stem]['fast']['TEA']['time'] = float(dataF[0])
        maps[m.stem]['robust']['TEA']['exp'] = float(dataR[1])
        maps[m.stem]['fast']['TEA']['exp'] = float(dataF[1])
        maps[m.stem]['robust']['TEA']['depth'] = float(dataR[2])
        maps[m.stem]['fast']['TEA']['depth'] = float(dataF[2])

        # PEA [time, avg exp, max depth]
        dataR = (timeRobustFile.readline().replace('\n', '')).split(' ')
        dataF = (timeFastFile.readline().replace('\n', '')).split(' ')
        maps[m.stem]['robust']['PEA']['time'] = float(dataR[0])
        maps[m.stem]['fast']['PEA']['time'] = float(dataF[0])
        maps[m.stem]['robust']['PEA']['exp'] = float(dataR[1])
        maps[m.stem]['fast']['PEA']['exp'] = float(dataF[1])
        maps[m.stem]['robust']['PEA']['depth'] = float(dataR[2])
        maps[m.stem]['fast']['PEA']['depth'] = float(dataF[2])
        counter = 0
        print(m.stem)
        robustTEA = []
        fastTEA = []
        robustPEA = []
        fastPEA = []
        robustE1 = []
        fastE1 = []
        robustE2 = []
        fastE2 = []
        robustE3 = []
        fastE3 = []
        while (True):
            counter += 1
            print(f'{counter/100}%', end='       \r')
            end = False
            for rl in robustLoggers:
                if not (rl.readline()):
                    end = True

            for fl in fastLoggers:
                if not (fl.readline()):
                    end = True
            if end:
                break

            points = []
            for l in robustLoggers:
                points.append(l.readline())

            for l in fastLoggers:
                points.append(l.readline())

            if any([points[0] != p for p in points]):
                print('comparing visibility from different points')
                exit()


            counts = []
            for l in robustLoggers:
                counts.append(l.readline())

            for l in fastLoggers:
                points.append(l.readline())

            robustVerts = []
            for l in robustLoggers:
                robustVerts.append(l.readline())

            fastVerts = []
            for l in fastLoggers:
                fastVerts.append(l.readline())

            robustPolys = [[], [], [], [], []]
            fastPolys = [[], [], [], [], []]

            for i in range(len(robustLoggers)):
                robustVerts[i] = np.array([eval(v) for v in robustVerts[i].split(';')[:-1]])
                if len(robustVerts[i]) == 0:
                    print('warning empty polygon')
                    exit()
                robustPolys[i] = geometry.Polygon([[p[0], p[1]] for p in robustVerts[i]])

            for i in range(len(fastLoggers)):
                fastVerts[i] = np.array([eval(v) for v in fastVerts[i].split(';')[:-1]])
                if len(fastVerts[i]) == 0:
                    print('warning empty polygon')
                    exit()
                fastPolys[i] = geometry.Polygon([[p[0], p[1]] for p in fastVerts[i]])

            robustTEA.append(robustPolys[0].area)
            fastTEA.append(abs(fastPolys[0].area - robustPolys[0].area))
            robustPEA.append(abs(robustPolys[1].area - robustPolys[0].area) / robustPolys[0].area)
            fastPEA.append(abs(fastPolys[1].area - robustPolys[1].area) / robustPolys[1].area)
            robustE1.append(abs(robustPolys[2].area - robustPolys[0].area) / robustPolys[0].area)
            fastE1.append(abs(fastPolys[2].area - robustPolys[2].area) / robustPolys[2].area)
            robustE2.append(abs(robustPolys[3].area - robustPolys[0].area) / robustPolys[0].area)
            fastE2.append(abs(fastPolys[3].area - robustPolys[3].area) / robustPolys[3].area)
            robustE3.append(abs(robustPolys[4].area - robustPolys[0].area) / robustPolys[0].area)
            fastE3.append(abs(fastPolys[4].area - robustPolys[4].area) / robustPolys[4].area)

        robustTEA = np.array(robustTEA)
        fastTEA = np.array(fastTEA)
        robustPEA = np.array(robustPEA)
        fastPEA = np.array(fastPEA)
        robustE1 = np.array(robustE1)
        fastE1 = np.array(fastE1)
        robustE2 = np.array(robustE2)
        fastE2 = np.array(fastE2)
        robustE3 = np.array(robustE3)
        fastE3 = np.array(fastE3)

        maps[m.stem]['robust']['EdgeVis1']['area'] = {}
        maps[m.stem]['fast']['EdgeVis1']['area'] = {}
        maps[m.stem]['robust']['EdgeVis2']['area'] = {}
        maps[m.stem]['fast']['EdgeVis2']['area'] = {}
        maps[m.stem]['robust']['EdgeVis3']['area'] = {}
        maps[m.stem]['fast']['EdgeVis3']['area'] = {}
        maps[m.stem]['robust']['TEA']['area'] = {}
        maps[m.stem]['fast']['TEA']['area'] = {}
        maps[m.stem]['robust']['PEA']['area'] = {}
        maps[m.stem]['fast']['PEA']['area'] = {}

        maps[m.stem]['robust']['EdgeVis1']['area']['mean'] = robustE1.mean()
        maps[m.stem]['fast']['EdgeVis1']['area']['mean'] = fastE1.mean()
        maps[m.stem]['robust']['EdgeVis2']['area']['mean'] = robustE2.mean()
        maps[m.stem]['fast']['EdgeVis2']['area']['mean'] = fastE2.mean()
        maps[m.stem]['robust']['EdgeVis3']['area']['mean'] = robustE3.mean()
        maps[m.stem]['fast']['EdgeVis3']['area']['mean'] = fastE3.mean()
        maps[m.stem]['robust']['TEA']['area']['mean'] = robustTEA.mean()
        maps[m.stem]['fast']['TEA']['area']['mean'] = fastTEA.mean()
        maps[m.stem]['robust']['PEA']['area']['mean'] = robustPEA.mean()
        maps[m.stem]['fast']['PEA']['area']['mean'] = fastPEA.mean()

        maps[m.stem]['robust']['EdgeVis1']['area']['max'] = robustE1.max()
        maps[m.stem]['fast']['EdgeVis1']['area']['max'] = fastE1.max()
        maps[m.stem]['robust']['EdgeVis2']['area']['max'] = robustE2.max()
        maps[m.stem]['fast']['EdgeVis2']['area']['max'] = fastE2.max()
        maps[m.stem]['robust']['EdgeVis3']['area']['max'] = robustE3.max()
        maps[m.stem]['fast']['EdgeVis3']['area']['max'] = fastE3.max()
        maps[m.stem]['robust']['TEA']['area']['max'] = robustTEA.max()
        maps[m.stem]['fast']['TEA']['area']['max'] = fastTEA.max()
        maps[m.stem]['robust']['PEA']['area']['max'] = robustPEA.max()
        maps[m.stem]['fast']['PEA']['area']['max'] = fastPEA.max()

        # map name
        detailsRobustSummary.readline()
        detailsFastSummary.readline()

        # vertices
        maps[m.stem]['vertices'] = int(detailsRobustSummary.readline().replace('\n', ''))
        detailsFastSummary.readline()

        # obstacles
        maps[m.stem]['obstacles'] = int(detailsRobustSummary.readline().replace('\n', ''))
        detailsFastSummary.readline()

        # triangles
        maps[m.stem]['triangles'] = int(detailsRobustSummary.readline().replace('\n', ''))
        detailsFastSummary.readline()

        # polygons
        maps[m.stem]['polygons'] = int(detailsRobustSummary.readline().replace('\n', ''))
        detailsFastSummary.readline()

        # edges precomp time
        detailsRobustSummary.readline()
        detailsFastSummary.readline()

        # evaluated edges
        maps[m.stem]['edges'] = {}
        maps[m.stem]['edges']['count'] = int(detailsRobustSummary.readline().replace('\n', ''))
        detailsFastSummary.readline()

        robustData = []
        fastData = []
        for i in range(maps[m.stem]['edges']['count']):
            robustData.append((detailsRobustSummary.readline().replace('\n', '')).split(';'))
            fastData.append((detailsFastSummary.readline().replace('\n', '')).split(';'))
            if robustData[-1] != fastData[-1]:
                print('warning')

        robustData = np.array(robustData).astype(float).T
        fastData = np.array(fastData).astype(float).T

        maps[m.stem]['edges']['expRight'] = {}
        maps[m.stem]['edges']['expLeft'] = {}
        maps[m.stem]['edges']['depthRight'] = {}
        maps[m.stem]['edges']['depthLeft'] = {}

        maps[m.stem]['edges']['expRight']['mean'] = robustData[2].mean()
        maps[m.stem]['edges']['expLeft']['mean'] = robustData[3].mean()
        maps[m.stem]['edges']['depthRight']['mean'] = robustData[0].mean()
        maps[m.stem]['edges']['depthLeft']['mean'] = robustData[1].mean()

        maps[m.stem]['edges']['expRight']['min'] = robustData[2].min()
        maps[m.stem]['edges']['expLeft']['min'] = robustData[3].min()
        maps[m.stem]['edges']['depthRight']['min'] = robustData[0].min()
        maps[m.stem]['edges']['depthLeft']['min'] = robustData[1].min()

        maps[m.stem]['edges']['expRight']['max'] = robustData[2].max()
        maps[m.stem]['edges']['expLeft']['max'] = robustData[3].max()
        maps[m.stem]['edges']['depthRight']['max'] = robustData[0].max()
        maps[m.stem]['edges']['depthLeft']['max'] = robustData[1].max()

        timeRobustFile.close()
        timeFastFile.close()
        detailsRobustSummary.close()
        detailsFastSummary.close()
        for l in robustLoggers:
            l.close()
        for l in fastLoggers:
            l.close()

    with open('saved_dictionary.pkl', 'wb') as f:
        pickle.dump(maps, f)


@cli.command('make-tables')
def make():
    with open('saved_dictionary.pkl', 'rb') as f:
        data = pickle.load(f)

        ################################################################
        #  table 1
        table = Path('TEA_PEA_times.tex')
        f = table.open('w')
        f.write('\\begin{table}\n')
        f.write('\\begin{center}\n')
        f.write('\\begin{tabular}{c@{\hskip .5cm}c c c@{\hskip 1cm}cc@{\hskip 1cm}c c}\n')
        f.write('\\toprule\n')
        f.write('\\multirow{2}{*}{map} & \multicolumn{3}{c}{runtime[s]} & \multicolumn{2}{c}{avg. exp.[-]} & \multicolumn{2}{c}{avg. depth[-]}\\\\\n')
        f.write('  & TEA & PEA & \\uparrow[\%] & TEA & PEA & TEA & PEA  \\\\\n')

        datatmp = [0, 0, 0, 0, 0, 0, 0, 0]
        ordered = list(data.keys())
        for map in sorted(ordered):
            name = str(map).replace('scene_', '')
            string =(f'{name} & {data[map]["robust"]["TEA"]["time"]:.2f} &'
                     f' {data[map]["robust"]["PEA"]["time"]:.2f}  &'
                     f' {100*(1-data[map]["robust"]["PEA"]["time"]/data[map]["robust"]["TEA"]["time"]):.1f}  &'
                     f' {data[map]["robust"]["TEA"]["exp"]:.0f}  &'
                     f' {data[map]["robust"]["PEA"]["exp"]:.0f}  &'
                     f' {data[map]["robust"]["TEA"]["depth"]:.0f}  &'
                     f' {data[map]["robust"]["PEA"]["depth"]:.0f}  \\\\\n').replace('_', '\_')
            datatmp[0] += 1
            datatmp[1] += data[map]["robust"]["TEA"]["time"]
            datatmp[2] += data[map]["robust"]["PEA"]["time"]
            datatmp[3] += 100*(1-data[map]["robust"]["PEA"]["time"]/data[map]["robust"]["TEA"]["time"])
            datatmp[4] += data[map]["robust"]["TEA"]["exp"]
            datatmp[5] += data[map]["robust"]["PEA"]["exp"]
            datatmp[6] += data[map]["robust"]["TEA"]["depth"]
            datatmp[7] += data[map]["robust"]["PEA"]["depth"]
            f.write(string)
        f.write('\\midrule\n')
        string = (f'Average &'
                  f' {datatmp[1]/datatmp[0]:.2f} &'
                  f' {datatmp[2]/datatmp[0]:.2f} &'
                  f' {datatmp[3]/datatmp[0]:.2f} &'
                  f' {datatmp[4]/datatmp[0]:.0f} &'
                  f' {datatmp[5]/datatmp[0]:.0f} &'
                  f' {datatmp[6]/datatmp[0]:.0f} &'
                  f' {datatmp[7]/datatmp[0]:.0f} \\\\\n').replace('_', '\_')
        f.write(string)
        f.write('\\bottomrule\n')
        f.write('\\end{tabular}\n')
        f.write('\\end{center}\n')
        f.write('\\caption{Comparison of TEA and PEA performance.}\n')
        f.write('\\end{table}\n')
        f.close()

        ################################################################
        #  table 2
        table = Path('IronHarvest.tex')
        f = table.open('w')
        f.write('\\begin{table}\n')
        f.write('\\begin{center}\n')
        f.write('\\begin{tabular}{c@{\hskip .5cm}c c c c}\n')
        f.write('\\toprule\n')
        f.write('map & obstacles & vertices & triangles & polygons \\\\\n')
        f.write('\\midrule\n')
        ordered = list(data.keys())
        for map in sorted(ordered):
            name = str(map).replace('scene_', '')
            string = (f'{name} &'
                      f' {data[map]["obstacles"]} &'
                      f' {data[map]["vertices"]} &'
                      f' {data[map]["triangles"]} &'
                      f' {data[map]["polygons"]} \\\\\n').replace('_', '\_')
            f.write(string)

        f.write('\\bottomrule\n')
        f.write('\\end{tabular}\n')
        f.write('\\end{center}\n')
        f.write('\\caption{Properties of maps in Iron Harvest dataset.}\n')
        f.write('\\end{table}\n')
        f.close()

        ################################################################
        #  table 3
        table = Path('EdgeVisibility.tex')
        f = table.open('w')
        f.write('\\begin{table}\n')
        f.write('\\begin{center}\n')
        f.write('\\begin{tabular}{c@{\hskip .5cm}c@{\hskip .5cm}c@{\hskip 1cm}c c@{\hskip 1cm} c c}\n')
        f.write('\\toprule\n')
        f.write('\\multirow{2}{*}{map} &  \multirow{2}{*}{count} & \multirow{2}{*}{runtime[s]} & \multicolumn{2}{c}{depth} & \multicolumn{2}{c}{expansions} \\\\\n')
        f.write(' & & & mean & max & mean & max \\\\\n')
        f.write('\\midrule\n')
        ordered = list(data.keys())
        for map in sorted(ordered):
            detailsRobustFolder = Path('./robust_details')
            detailsFastFolder = Path('./nonrobust_details')
            detailsRobustMapFolder = (detailsRobustFolder / f'{map}')
            detailsFastMapFolder = (detailsFastFolder / f'{map}')
            detailsRobustSummary = (detailsRobustMapFolder / 'sideResults.dat').open('r')
            detailsFastSummary = (detailsFastMapFolder / 'sideResults.dat').open('r')

            # map name
            detailsRobustSummary.readline()
            detailsFastSummary.readline()
            # vertices
            detailsRobustSummary.readline()
            detailsFastSummary.readline()
            # obstacles
            detailsRobustSummary.readline()
            detailsFastSummary.readline()
            # triangles
            detailsRobustSummary.readline()
            detailsFastSummary.readline()
            # polygons
            detailsRobustSummary.readline()
            detailsFastSummary.readline()

            # edges precomp time
            detailsRobustSummary.readline()
            detailsFastSummary.readline()

            # evaluated edges
            count = int(detailsRobustSummary.readline().replace('\n', ''))
            detailsFastSummary.readline()

            robustData = []
            fastData = []
            for i in range(count):
                robustData.append((detailsRobustSummary.readline().replace('\n', '')).split(';'))
                fastData.append((detailsFastSummary.readline().replace('\n', '')).split(';'))
                if robustData[-1] != fastData[-1]:
                    print('warning')

            robustData = np.array(robustData).astype(float).T
            fastData = np.array(fastData).astype(float).T

            depth = np.max(robustData[0:1,:], axis=0)
            exp = np.sum(robustData[2:3,:], axis=0)

            name = str(map).replace('scene_', '')
            string = (f'{name} &'
                      f' {data[map]["edges"]["count"]} &'
                      f' {data[map]["robust"]["searchnodes"]:.3f} &'
                      f' {depth.mean():.0f} &'
                      f' {depth.max():.0f} &'
                      f' {exp.mean():.0f} &'
                      f' {exp.max():.0f}  \\\\\n').replace('_', '\_')
            f.write(string)

        f.write('\\bottomrule\n')
        f.write('\\end{tabular}\n')
        f.write('\\end{center}\n')
        f.write('\\caption{Computation of visibility regions of traversable edges in a triangular mesh.}\n')
        f.write('\\end{table}\n')
        f.close()

        if True:
            ################################################################
            #  table 4
            table = Path('PEA_EdgeVis_times.tex')
            f = table.open('w')
            f.write('\\begin{table}\n')
            f.write('\\begin{center}\n')
            f.write('\\begin{tabular}{c@{\hskip .5cm}c c c@{\hskip 1cm}c c c{\hskip .1cm}c}\n')
            f.write('\\toprule\n')
            f.write(
                '\\multirow{2}{*}{map} & \multicolumn{3}{c}{\\uparrow[\%]} & \multicolumn{3}{c}{evaluated v.} & \multirow{2}{*}{E3 \\checkmark[\%]} \\\\\n')
            f.write('  & E1 & E2 & E3 & E1 & E2 & E3 &  \\\\\n')
            f.write('\\midrule\n')

            ordered = list(data.keys())
            datatmp = [0, 0 ,0 ,0 ,0 ,0 ,0, 0]
            for map in sorted(ordered):
                correct = analyze_map(map, 1, 2, False)
                print(map)
                if correct[0] < 10000 or correct[1] < 10000 or correct[2] < 10000:
                    print('Badly evald', correct)
                name = str(map).replace('scene_', '')
                string = (f'{name} &'
                          f' {100 * (1 - data[map]["robust"]["EdgeVis1"]["time"] / data[map]["robust"]["PEA"]["time"]):.1f} &'
                          f' {100 * (1 - data[map]["robust"]["EdgeVis2"]["time"] / data[map]["robust"]["PEA"]["time"]):.1f} &'
                          f' {100 * (1 - data[map]["robust"]["EdgeVis3"]["time"] / data[map]["robust"]["PEA"]["time"]):.1f} &'
                          f' { data[map]["robust"]["EdgeVis1"]["evald"]:.0f} &'
                          f' { data[map]["robust"]["EdgeVis2"]["evald"]:.0f} &'
                          f' { data[map]["robust"]["EdgeVis3"]["evald"]:.0f} &'
                          f' { 100*correct[3]/correct[0]:.2f}  \\\\\n').replace('_', '\_')
                datatmp[0] += 1
                datatmp[1] += 100 * (1 - data[map]["robust"]["EdgeVis1"]["time"] / data[map]["robust"]["PEA"]["time"])
                datatmp[2] += 100 * (1 - data[map]["robust"]["EdgeVis2"]["time"] / data[map]["robust"]["PEA"]["time"])
                datatmp[3] += 100 * (1 - data[map]["robust"]["EdgeVis3"]["time"] / data[map]["robust"]["PEA"]["time"])
                datatmp[4] += data[map]["robust"]["EdgeVis1"]["evald"]
                datatmp[5] += data[map]["robust"]["EdgeVis2"]["evald"]
                datatmp[6] += data[map]["robust"]["EdgeVis3"]["evald"]
                datatmp[7] += 100*correct[3]/correct[0]
                f.write(string)
            f.write('\\midrule\n')
            string = (f'Average &'
                      f' {datatmp[1]/datatmp[0]:.1f} &'
                      f' {datatmp[2]/datatmp[0]:.1f} &'
                      f' {datatmp[3]/datatmp[0]:.1f} &'
                      f' {datatmp[4]/datatmp[0]:.0f} &'
                      f' {datatmp[5]/datatmp[0]:.0f} &'
                      f' {datatmp[6]/datatmp[0]:.0f} &'
                      f' {datatmp[7]/datatmp[0]:.2f} \\\\\n').replace('_', '\_')
            f.write(string)

            f.write('\\bottomrule\n')
            f.write('\\end{tabular}\n')
            f.write('\\end{center}\n')
            f.write('\\caption{Comparison of EdgeVis variants with PEA.}\n')
            f.write('\\end{table}\n')
            f.close()

        ################################################################
        #  table 5
        table = Path('Preprocessing.tex')
        f = table.open('w')
        f.write('\\begin{table}\n')
        f.write('\\begin{center}\n')
        f.write('\\begin{tabular}{c@{\hskip .5cm}c c c c c c}\n')
        f.write('\\toprule\n')

        f.write('map & CDT[ms] & M-CDT[ms] & edges[ms] & E1[ms] & E2[ms] & E3[ms] \\\\\n')
        f.write('\\midrule\n')

        ordered = list(data.keys())
        for map in sorted(ordered):
            name = str(map).replace('scene_', '')
            string = (f'{name} &'
                      f' {1000*data[map]["robust"]["CDT"]:.0f} &'
                      f' {1000*data[map]["robust"]["M-CDT"]:.0f} &'
                      f' {1000*data[map]["robust"]["searchnodes"]:.0f} &'
                      f' {1000*data[map]["robust"]["optimnodes1"]:.0f} &'
                      f' {1000*(data[map]["robust"]["optimnodes2"]):.0f} &'
                      f' {1000*(data[map]["robust"]["optimnodes3"]):.0f}  \\\\\n').replace('_', '\_')

            f.write(string)


        f.write('\\bottomrule\n')
        f.write('\\end{tabular}\n')
        f.write('\\end{center}\n')
        f.write('\\caption{Table of preprocessing times for all algorithms.}\n')
        f.write('\\end{table}\n')
        f.close()

        ################################################################
        #  table 6
        table = Path('NotRobust.tex')
        f = table.open('w')
        f.write('\\begin{table}\n')
        f.write('\\begin{center}\n')
        f.write('\\begin{tabular}{c@{\hskip .5cm}c c c c c}\n')
        f.write('\\toprule\n')
        f.write('map & TEA[\%] & PEA[\%] & E1[\%] & E2[\%] & E3[\%] \\\\\n')
        f.write('\\midrule\n')
        datatmp = [0, 0, 0, 0, 0, 0]
        ordered = list(data.keys())
        for map in sorted(ordered):
            name = str(map).replace('scene_', '')
            string = (f'{name} &'
                      f' {100 * (1-data[map]["fast"]["TEA"]["time"]/data[map]["robust"]["TEA"]["time"]):.2f} &'
                      f' {100 * (1-data[map]["fast"]["PEA"]["time"]/data[map]["robust"]["PEA"]["time"]):.2f} &'
                      f' {100 * (1-data[map]["fast"]["EdgeVis1"]["time"]/data[map]["robust"]["EdgeVis1"]["time"]):.2f} &'
                      f' {100 * (1-data[map]["fast"]["EdgeVis2"]["time"]/data[map]["robust"]["EdgeVis2"]["time"]):.2f} &'
                      f' {100 * (1-data[map]["fast"]["EdgeVis3"]["time"]/data[map]["robust"]["EdgeVis3"]["time"]):.2f} \\\\\n').replace('_', '\_')
            datatmp[0] += 1
            datatmp[1] += 100 * (1-data[map]["fast"]["TEA"]["time"]/data[map]["robust"]["TEA"]["time"])
            datatmp[2] += 100 * (1-data[map]["fast"]["PEA"]["time"]/data[map]["robust"]["PEA"]["time"])
            datatmp[3] += 100 * (1-data[map]["fast"]["EdgeVis1"]["time"]/data[map]["robust"]["EdgeVis1"]["time"])
            datatmp[4] += 100 * (1-data[map]["fast"]["EdgeVis2"]["time"]/data[map]["robust"]["EdgeVis2"]["time"])
            datatmp[5] += 100 * (1-data[map]["fast"]["EdgeVis3"]["time"]/data[map]["robust"]["EdgeVis3"]["time"])
            f.write(string)
        f.write('\\midrule\n')
        string = (f'Average &'
                      f' {datatmp[1] / datatmp[0]:.2f} &'
                      f' {datatmp[2] / datatmp[0]:.2f} &'
                      f' {datatmp[3] / datatmp[0]:.2f} &'
                      f' {datatmp[4] / datatmp[0]:.2f} &'
                      f' {datatmp[5] / datatmp[0]:.2f} \\\\\n').replace('_', '\_')
        f.write(string)

        f.write('\\bottomrule\n')
        f.write('\\end{tabular}\n')
        f.write('\\end{center}\n')
        f.write('\\caption{Improvement in query the performance without usage of robust orientation predicates.}\n')
        f.write('\\end{table}\n')
        f.close()


@cli.command('analyze-maps')
@click.option('-n', '--number', type=int, default=10000)
@click.option('-r', '--robust', is_flag=True)
def analyze(number, robust):
    maps = Path("../dependencies/IronHarvest/mesh-maps/iron-harvest")
    if robust:
        saveDir = Path(f'robust_details')
    else:
        saveDir = Path(f'nonrobust_details')

    saveDir.mkdir(exist_ok=True)

    for m in maps.iterdir():
        if (m.suffix != '.mesh'):
            continue
        name = m.stem
        print(name)
        try:
            analyze_map(name, number, robust)
        except Exception as e:
            print(f'failed {repr(e)}')
            print("-" * 50)


def analyze_map(map, number, robust, flag=True):

    name = map
    if flag:
        executable = f"../build/source/EdgeVis_example"
        if robust:
            saveDir = Path(f'robust_details')
        else:
            saveDir = Path(f'nonrobust_details')

        saveDir.mkdir(exist_ok=True)
        saveDir = saveDir / name
        saveDir.mkdir(exist_ok=True)

        if robust:
            arguments = f' -m {name} -n {str(number)} --random-seed 1 --save -r'
        else:
            arguments = f' -m {name} -n {str(number)} --random-seed 1 --save'
        command = executable + arguments
        subprocess.run(command, shell=True, stdout=subprocess.DEVNULL)
        bad = []
        try:
            os.replace("./sideResults.dat", f'{str(saveDir.resolve())}/sideResults.dat')
            os.replace("./E1.log", f'{str(saveDir.resolve())}/E1.log')
            os.replace("./E2.log", f'{str(saveDir.resolve())}/E2.log')
            os.replace("./E3.log", f'{str(saveDir.resolve())}/E3.log')
            os.replace("./PEA_polygon.log", f'{str(saveDir.resolve())}/PEA_polygon.log')
            os.replace("./PEA_triangl.log", f'{str(saveDir.resolve())}/PEA_triangl.log')
        except FileNotFoundError:
            bad.append(name)
            print(f"{name} is not usable")
        if len(bad) > 0:
            print(bad)
        return [0, 0, 0, 0]

    else:
        if robust:
            saveDir = Path(f'robust_details')
        else:
            saveDir = Path(f'nonrobust_details')

        saveDir.mkdir(exist_ok=True)
        saveDir = saveDir / name
        saveDir.mkdir(exist_ok=True)

        e1Path = Path(f'{str(saveDir.resolve())}/E1.log')
        e2Path = Path(f'{str(saveDir.resolve())}/E2.log')
        e3Path = Path(f'{str(saveDir.resolve())}/E3.log')
        PPPath = Path(f'{str(saveDir.resolve())}/PEA_polygon.log')
        PTPath = Path(f'{str(saveDir.resolve())}/PEA_triangl.log')


        res = [0,0,0]
        e1 = e1Path.open('r')
        e2 = e2Path.open('r')
        e3 = e3Path.open('r')
        PT = PTPath.open('r')
        PP = PPPath.open('r')
        logs = [PT, PP, e1, e2, e3]

        data = [0, 0, 0, 0]
        counter = 0
        ctr = 0
        evald = 0
        while(True):
            try:
                counter += 1
                print(f'{counter / 100}%', end='       \r')
                # --
                end = False
                for l in logs:
                    if not (l.readline()):
                        end = True
                if end:
                    break
                ctr += 1
                # seeker

                points = []
                for l in logs:
                    points.append(l.readline())

                counts = []
                for l in logs:
                    counts.append(l.readline())

                verts = []
                for l in logs:
                    verts.append(l.readline())

                polys = [[],[],[],[],[]]
                for i in range(len(logs)):
                    verts[i] = np.array([eval(v) for v in verts[i].split(';')[:-1]])
                    if len(verts[i]) == 0:
                        end = True
                    polys[i] = geometry.Polygon([[p[0], p[1]] for p in verts[i]])
                if end:
                    continue
                evald += 1

                if abs(polys[0].area - polys[1].area)< 1e-6:
                    data[0] += 1

                if abs(polys[0].area - polys[2].area)< 1e-6:
                    data[1] += 1

                if abs(polys[0].area - polys[3].area)< 1e-6:
                    data[2] += 1

                if abs(polys[0].area - polys[4].area)< 1e-6:
                    data[3] += 1

            except NameError:
                print("nan")
                res[2] += 1
                pass
        e1.close()
        e2.close()
        e3.close()
        PT.close()
        PP.close()
        data = np.array(data)

        return data


if __name__ == "__main__":
    cli()
