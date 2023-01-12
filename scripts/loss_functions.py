import numpy as np  

# def aux_haversine(lon1, lat1, lon2, lat2): # haversine is the angle distance between two points on a sphere 
    
#     from math import radians, cos, sin, asin, sqrt

#     R = 6372.8

#     dLat = radians(lat2 - lat1)
#     dLon = radians(lon2 - lon1)
#     lat1 = radians(lat1)
#     lat2 = radians(lat2)

#     a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
#     c = 2*asin(sqrt(a))

#     return R * c


# def aux_CalculateErrorSinglePoint( trajectory, targetPoint ):
    
#     # np.seterr('raise')

#     # da napravam mapa so traektorija so markeri za targetpointite so ke se presmetva zaguba
#     # map_error_points(trajectory, targetPoint)

#     v_m1 = aux_haversine( trajectory[ targetPoint - 1, 2 ], trajectory[ targetPoint - 1, 1 ], trajectory[ targetPoint, 2 ], trajectory[ targetPoint, 1 ] )
#     if v_m1 == 0:
#         print(f'Minus 1 targetpoint: {targetPoint}')
#         print(f'Latitude1: {trajectory[targetPoint-1][1]}. Longitude1: {trajectory[targetPoint-1][2]}')
#         print(f'Latitude2: {trajectory[targetPoint][1]}. Longitude2: {trajectory[targetPoint][2]}')
#         print()
#     # s_m1 = v_m1 / ( trajectory[ targetPoint, 0 ] - trajectory[ targetPoint - 1, 0 ] )
    
    
#     v_m2 = aux_haversine( trajectory[ targetPoint - 2, 2 ], trajectory[ targetPoint - 2, 1 ], trajectory[ targetPoint - 1, 2 ], trajectory[ targetPoint - 1, 1 ] )
#     if v_m2 == 0:
#         print(f'Minus 2 targetpoint: {targetPoint}')
#         print(f'Latitude1: {trajectory[targetPoint-2][1]}. Longitude1: {trajectory[targetPoint-2][2]}')
#         print(f'Latitude2: {trajectory[targetPoint-1][1]}. Longitude2: {trajectory[targetPoint-1][2]}')
#         print()
#     # s_m2 = v_m2 / ( trajectory[ targetPoint - 1, 0 ] - trajectory[ targetPoint - 2, 0 ] )
    
    
#     v_p1 = aux_haversine( trajectory[ targetPoint, 2 ], trajectory[ targetPoint, 1 ], trajectory[ targetPoint + 1, 2 ], trajectory[ targetPoint + 1, 1 ] )
#     if v_p1 == 0:
#         print(f'Plus 1 targetpoint: {targetPoint}')
#         print(f'Latitude1: {trajectory[targetPoint+1][1]}. Longitude1: {trajectory[targetPoint+1][2]}')
#         print(f'Latitude2: {trajectory[targetPoint][1]}. Longitude2: {trajectory[targetPoint][2]}')
#         print()
#     # s_p1 = v_p1 / ( trajectory[ targetPoint + 1, 0 ] - trajectory[ targetPoint, 0 ] )
    
    
#     v_p2 = aux_haversine( trajectory[ targetPoint + 1, 2 ], trajectory[ targetPoint + 1, 1 ], trajectory[ targetPoint + 2, 2 ], trajectory[ targetPoint + 2, 1 ] )
#     if v_p2 == 0:
#         print(f'Plus 2 targetpoint: {targetPoint}')
#         print(f'Latitude1: {trajectory[targetPoint+2][1]}. Longitude1: {trajectory[targetPoint+2][2]}')
#         print(f'Latitude2: {trajectory[targetPoint+1][1]}. Longitude2: {trajectory[targetPoint+1][2]}')
#         print()
#     # s_p2 = v_p2 / ( trajectory[ targetPoint + 2, 0 ] - trajectory[ targetPoint + 1, 0 ] )

#     return np.std( [ v_m2, v_m1, v_p1, v_p2 ] ) / np.mean( [ v_m2, v_m1, v_p1, v_p2 ] )



# def aux_CalculateTrajectoryError( trajectory, variablePoints ):
    
#     totalError = 0.0
    
#     for k in np.where(variablePoints)[0]:
        
#         single_point_error = aux_CalculateErrorSinglePoint( trajectory, k )
        
#         if ~np.isnan(single_point_error):

#             totalError += single_point_error
    
#     return totalError



globalSpeedWeight = 0.05
globalPower = 1.0

def aux_haversine(lon1, lat1, lon2, lat2): # haversine is the angle distance between two points on a sphere 
    
    from math import radians, cos, sin, asin, sqrt

    R = 6372.8

    dLat = radians(lat2 - lat1)
    dLon = radians(lon2 - lon1)
    lat1 = radians(lat1)
    lat2 = radians(lat2)

    a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
    c = 2*asin(sqrt(a))

    return R * c





def aux_CalculateErrorSinglePoint( trajectory: np.array, targetPoint ):
    
    segmDist = []
    segmSpeed = []
    for k in range( -2, 2 ):
        d = aux_haversine( trajectory[ targetPoint + k, 2 ], trajectory[ targetPoint + k, 1 ], \
                           trajectory[ targetPoint + k + 1, 2 ], trajectory[ targetPoint + k + 1, 1 ] )
        s = d / ( trajectory[ targetPoint + k + 1, 0 ] - trajectory[ targetPoint + k, 0 ] )
        segmDist.append( d )
        segmSpeed.append( s )
            
    segmSpeedD = np.array( segmSpeed )
    segmSpeedD = segmSpeedD[ 1: ] - segmSpeedD[ :-1 ]
    speedError = np.std( segmSpeedD ) / np.mean( segmSpeed )
    
    segmAngles = []
    for k in range( -1, 2 ):
        v1 = [ trajectory[ targetPoint + k + 1, 2 ] - trajectory[ targetPoint + k, 2 ], \
               trajectory[ targetPoint + k + 1, 1 ] - trajectory[ targetPoint + k, 1 ] ]
        v1 = v1 / np.linalg.norm( v1 )
        v2 = [ trajectory[ targetPoint + k, 2 ] - trajectory[ targetPoint + k - 1, 2 ], \
               trajectory[ targetPoint + k, 1 ] - trajectory[ targetPoint + k - 1, 1 ] ]
        v2 = v2 / np.linalg.norm( v2 )

        dot_product = np.dot( v1, v2 )
        angle = np.arccos( dot_product )
        segmAngles.append( angle / ( trajectory[ targetPoint + k + 1, 0 ] - trajectory[ targetPoint + k - 1, 0 ] ) )
        
    segmAnglesD = np.array( segmAngles )
    segmAnglesD = segmAnglesD[ 1: ] - segmAnglesD[ :-1 ]
    angleError = np.std( segmAnglesD ) / np.mean( segmAngles )
    
    speedError = np.power( speedError, globalPower )
    angleError = np.power( angleError, globalPower )
    
    totalError = globalSpeedWeight * speedError + ( 1.0 - globalSpeedWeight ) * angleError
#    if np.isnan( totalError ): totalError = 999.9
    
    return totalError

    




def aux_CalculateTrajectoryError( trajectory, variablePoints ):
    
    totalError = 0.0
    
    for k in range( 2, np.size( variablePoints ) - 2 ):
        
        if variablePoints[ k ]:
            single_point_error = aux_CalculateErrorSinglePoint( trajectory, k )
            
            if ~np.isnan(single_point_error):

                totalError += single_point_error
    
    return totalError


