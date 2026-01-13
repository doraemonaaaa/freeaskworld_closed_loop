

def world_to_map(map_info, x, y):
    if map_info is None: 
        return 0, 0
    origin = map_info.origin.position
    res = map_info.resolution
    mx = int((x - origin.x) / res)
    my = int((y - origin.y) / res)
    return mx, my