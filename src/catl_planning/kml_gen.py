# DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

# This material is based upon work supported by the Under Secretary of Defense for 
# Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any 
# opinions, findings, conclusions or recommendations expressed in this material 
# are those of the author(s) and do not necessarily reflect the views of the Under 
# Secretary of Defense for Research and Engineering.

# (C) 2021 Massachusetts Institute of Technology.

# Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

# The software/firmware is provided to you on an As-Is basis

# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS 
# Part 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, U.S. 
# Government rights in this work are defined by DFARS 252.227-7013 or DFARS 
# 252.227-7014 as detailed above. Use of this work other than as specifically 
# authorized by the U.S. Government may violate any copyrights that exist in this 
# work.

# SPDX-License-Identifier: BSD-3-Clause

import numpy as np
import scipy as sp
import os
import simplekml as simplekml
from simplekml import Kml, Snippet, Types

from datetime import datetime
from datetime import timedelta
from lomap.classes.ts import Ts
###############################################################################
#This class definition creates a struct target_data structure
class empty_struct:
	pass

##############################################################################

def gen_kml_file(m,plan_to_draw):

    #points = empty_struct()
    new_agent_names = [["A1"],["A2"],["A3"],["A4"],["B1"],["B2"],["B3"],["B4"],["C1"],["C2"],["C3"]]
    agent_names = ["0","1","2","3","4","5","6","7","8","9","10"]
    agent_color = ['ff00ff00','ff00ff00','ff00ff00','ff00ff00','551400ff','551400ff','551400ff','551400ff','55000000','55000000','55000000']
    points = empty_struct()
    num_agents = 11
    time_step_size = (30*60)/(150*12)
    corner_coor = m.corner_coor
    alt = 500
    scale_to_m_y = 85.8000#71.666667
    scale_to_m_x = 73.0000#52.142857
    start_time = datetime.now()

    # Create the KML document
    kml = Kml(name="CaTL", open=1)
    doc = kml.newdocument(name='test')
    doc.lookat.longitude = -71.0000000000
    doc.lookat.latitude = 42.33
    doc.lookat.range = 5000.00

    # Create a folder
    fol = doc.newfolder(name='Tracks')


    #Get the region points and convert them to lat_long
    ts = Ts.load(m.ts_filename)
    region_bounds = []
    region_color = []
    region_label = []
    num_states = 0
    value = 'pol'

    for state in ts.g.node:
            lat_long_bounds = []
            state_int = np.int(state[1:len(state)])
            bounds = ts.g.node[state]['shape']
            prop = ts.g.node[state]['prop']
            prop_str = str(prop)
            len_b = len(prop_str)
            label = prop_str[6:len_b-3]
            color = ts.g.node[state]['color']
            num_states += 1
            num_points = len(bounds)
            for i in range(0,num_points):
                lat,long = point_transform((bounds[i][0],bounds[i][1]),corner_coor,scale_to_m_x,scale_to_m_y)
                lat_long_bounds.append((long,lat,500))
            pol_var = value+str(state_int)
            #Generate polygon map of regions:
            vars()[pol_var] = kml.newpolygon(name=label,
                             outerboundaryis=lat_long_bounds)
            if color == 'white':
                vars()[pol_var].style.polystyle.color = '5514ffff'
            if color == 'yellow':
                vars()[pol_var].style.polystyle.color = '5514f0ff'
            if color == 'red':
                vars()[pol_var].style.polystyle.color = '551400ff'
            if color == 'black':
                vars()[pol_var].style.polystyle.color = '55140000'

            vars()[pol_var].style.polystyle.outline = 1
            vars()[pol_var].style.labelstyle.color = simplekml.Color.white
            #vars()[pol_var].gxtimespan.begin = current_time
            #vars()[pol_var].gxtimespan.end = current_time + timedelta(seconds=time_step_size)

    #Loop Starts here:

    for agent in range(0,plan_to_draw.shape[0]):
        label_name = new_agent_names[int(agent)][0]
        agent_color_val = agent_color[int(agent)]
        #agent = agent_name
        points.pts = []
        points.when = []

        cx = []
        cy = []
        for t in range(0,plan_to_draw.shape[1]):
            cx = list(cx)
            cy = list(cy)
            cx.append(plan_to_draw[agent][t][0])
            cy.append(plan_to_draw[agent][t][1])
            
        points.x = [i for i in cx if i>=0]
        points.y = [i for i in cy if i>=0]

        current_time = start_time + timedelta(seconds=time_step_size)

        #print(agent)
        
        #if len(points.x) > 1000:
         #   save_length = int(len(points.x)*.6)
        #else:
        #    save_length = int(len(points.x))
        for i in range(0, len(points.x)):
	        x = points.x[i]
	        #print(x)
	        y = points.y[i]
	        #print(y)
	        lat,long = point_transform((x,y),corner_coor,scale_to_m_x,scale_to_m_y)
	        points.pts.append([long,lat,alt])
	        current_time = current_time + timedelta(seconds=time_step_size)
	        points.when.append(current_time.strftime("%Y-%m-%d")+"T"+current_time.strftime("%H:%M:%S")+"Z")


        agent_info = {"coord" : points.pts,
                   	"when" : points.when
        }


        # Create the trajectory
        trk = fol.newgxtrack(name=label_name)
        trk.newwhen(agent_info["when"])
        trk.newgxcoord(agent_info["coord"])
        trk.altitudemode = 'relativeToGround'
        # Styling
        trk.stylemap.normalstyle.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/shapes/airports.png'
        trk.stylemap.normalstyle.linestyle.color = '00000000'
        trk.stylemap.normalstyle.linestyle.width = 6
        trk.stylemap.highlightstyle.iconstyle.icon.href = 'http://maps.google.com/mapfiles/kml/shapes/airports.png'
        trk.stylemap.highlightstyle.iconstyle.scale = 1.2
        trk.stylemap.highlightstyle.linestyle.color = agent_color_val
        trk.stylemap.highlightstyle.linestyle.width = 8



    # Saving
    kml.save(m.save_filename+".kml")
    #kml.savekmz(os.path.splitext(__file__)[0] + ".kmz") # uncomment to save to kmz
    #print kml.kml() # uncomment to see the kml printed to screen
    

def txt_output(m,the_plan): 
    
    for a in range(0,len(the_plan)):
        valsx = []
        valsy = []   
        for t in range(0,len(the_plan[a])):
            valsx.append(the_plan[a][t][0])
            valsy.append(the_plan[a][t][1])
		    
        np.savetxt(m.output_path+'/txt_files/'+str(a)+'x.txt',valsx)
        np.savetxt(m.output_path+'/txt_files/'+str(a)+'y.txt',valsy)
        

def point_transform(point,origin,scale_to_m_x,scale_to_m_y,aprox_m_per_deg=111134.75):
	long = np.divide(point[0]*scale_to_m_x,aprox_m_per_deg)+origin[1]
	lat = np.divide(point[1]*scale_to_m_y,aprox_m_per_deg)+origin[0]
	return(lat,long)
