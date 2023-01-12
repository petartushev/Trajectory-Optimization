from django.shortcuts import render
from django.http import HttpResponse, JsonResponse
from pathlib import Path
import json
import sys
import re
import os
import numpy as np
import pandas as pd
import folium

BASE_DIR = Path(__file__).resolve().parent.parent

scripts_path = str(BASE_DIR.parent) + '/scripts/'

sys.path.insert(0, scripts_path)

import TrajectoryImprovementLibrary as TIL

# print(os.getcwd())


# print(TIL.__path__)




# Create your views here.


def map(request):


    lat, lon = 41.987734, 21.428074
    zoom_start = 6

    is_optimize_request = request.headers.get('optimize')

    is_ajax = request.headers.get('X-Requested-With') == 'XMLHttpRequest'

    if is_optimize_request:
        
        if is_ajax:
            if request.method == 'POST':

                print('OPTIMIZE CALLED')

                data = json.load(request)

                trajectory_path = str(BASE_DIR.parent) + f'/SampleTraj/Trj_{int(data["num"])}.npy'

                traj = np.load(trajectory_path)

                traj = pd.DataFrame(traj)
                traj.columns = ['timestamp', 'latitude', 'longitude', 'altitude']
                traj = traj.drop_duplicates(subset=['latitude', 'longitude'], keep='first').to_numpy()
                
                # --------------- HARDCODED VALUES FOR TRAJECTORY 1 ---------------

                targetPoints = [260]
                variablePoints = np.zeros( ( np.size( traj, 0 ) ), dtype = bool )
                variablePoints[targetPoints] = True

                # --------------- HARDCODED VALUES FOR TRAJECTORY 1 ---------------

                correctedTraj1 = TIL.TrajectoryImprovement( traj, variablePoints, data['algorithm'] )

                tmp = traj[:, 1:3]
                cTraj1 = correctedTraj1[:, 1:3]

                lat = np.mean([row[0] for row in tmp])
                lon = np.mean([row[1] for row in tmp])

                m = folium.Map(
                    location=[lat, lon],
                    tiles="cartodbpositron",
                    zoom_start=zoom_start
                )

                aline1 = folium.PolyLine(tmp, color='blue')
                m.add_child(aline1)

                aline2 = folium.PolyLine(cTraj1, color='red')
                m.add_child(aline2)

                print('RETURNED')
                return JsonResponse({'new_map': m._repr_html_()}, safe=True)
    else:

        if is_ajax:
            if request.method == 'POST':

                data = json.load(request)

                trajectory_path = str(BASE_DIR.parent) + f'/SampleTraj/Trj_{int(data["num"])}.npy'

                traj = np.load(trajectory_path)

                traj = pd.DataFrame(traj)
                traj.columns = ['timestamp', 'latitude', 'longitude', 'altitude']
                traj = traj.drop_duplicates(subset=['latitude', 'longitude'], keep='first').to_numpy()

                tmp = traj[:, 1:3]
                
                lat = np.mean([row[0] for row in tmp])
                lon = np.mean([row[1] for row in tmp])

                m = folium.Map(
                    location=[lat, lon],
                    tiles="cartodbpositron",
                    zoom_start=zoom_start
                )

                aline1 = folium.PolyLine(tmp, color='blue')
                m.add_child(aline1)

                return JsonResponse({'new_map': m._repr_html_()}, safe=True)

    all_trajectories = []
    traj_path = str(BASE_DIR.parent) + '/SampleTraj/'
    regex = re.compile('^Trj_.*.npy$')

    for _, _, files in os.walk(traj_path):
        for file in files:
            if regex.match(file):
                all_trajectories.append(file)

    # print(all_trajectories)

    m = folium.Map(
        location=[lat, lon],
        tiles="cartodbpositron",
        zoom_start=zoom_start
    )

    context = {
        'map': m._repr_html_(),
        'trajectories': all_trajectories
    }

    return render(request, 'map/base.html', context)

def optimize(request):



    lat, lon = 41.987734, 21.428074
    zoom_start = 6

    is_ajax = request.headers.get('X-Requested-With') == 'XMLHttpRequest'

    if is_ajax:
        if request.method == 'POST':

            print('OPTIMIZE CALLED')

            data = json.load(request)

            trajectory_path = str(BASE_DIR.parent) + f'/SampleTraj/Trj_{int(data["trajectories-select"])}.npy'

            traj = np.load(trajectory_path)

            traj = pd.DataFrame(traj)
            traj.columns = ['timestamp', 'latitude', 'longitude', 'altitude']
            traj = traj.drop_duplicates(subset=['latitude', 'longitude'], keep='first').to_numpy()
            
            # --------------- HARDCODED VALUES FOR TRAJECTORY 1 ---------------

            targetPoints = [260]
            variablePoints = np.zeros( ( np.size( traj, 0 ) ), dtype = bool )
            variablePoints[targetPoints] = True

            # --------------- HARDCODED VALUES FOR TRAJECTORY 1 ---------------

            # correctedTraj1 = TIL.TrajectoryImprovement( traj, variablePoints, data['algorithms-select'] )

            tmp = traj[:, 1:3]
            # cTraj1 = correctedTraj1[:, 1:3]

            lat = np.mean([row[0] for row in tmp])
            lon = np.mean([row[1] for row in tmp])

            m = folium.Map(
                location=[lat, lon],
                tiles="cartodbpositron",
                zoom_start=zoom_start
            )

            aline1 = folium.PolyLine(tmp, color='blue')
            m.add_child(aline1)

            # aline2 = folium.PolyLine(cTraj1, color='red')
            # m.add_child(aline2)

            print('RETURNED')
            return JsonResponse({'new_map': m._repr_html_()}, safe=True)

