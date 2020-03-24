def generate_discrete_path_from_points(p1, p2):
    # for a direct, continues path, generate a zig-zag path that follows the grid, which 
    # can be used for probability computation
    p1x = round(p1.x)
    p1y = round(p1.y)
    p2x = round(p2.x)
    p2y = round(p2.y)
    case = 0
    if p1x <= p2x and p1y <= p2y:   
        case = 1
    elif p1x >= p2x and p1y <= p2y:
        case = 2
    elif p1x <= p2x and p1y >= p2y:
        case = 3
    else:
        case = 4
    path = [[]]
    path[0] = [p1x, p1y]
    while(path[-1] != [p2x,p2y]):
        blocks = generate_next_block_candidates(path[-1], case)
        d0 = distance_between_line_and_point([p1x,p1y],[p2x,p2y],blocks[0])
        d1 = distance_between_line_and_point([p1x,p1y],[p2x,p2y],blocks[1])
        if d0 <= d1:
            path.append(blocks[0])
        else:
            path.append(blocks[1])
    return path

def distance_between_line_and_point(_p1, _p2, _p3):
    # called in generate_discrete_path_from_points()
    p1 = np.asarray(_p1)
    p2 = np.asarray(_p2)
    p3 = np.asarray(_p3)
    d = np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)
    return d

def generate_next_block_candidates(currentBlock, caseNum):
    # called in generate_discrete_path_from_points
    # returns a size-2 array, containing two candidate of the next blocks
    x0 = currentBlock[0]
    y0 = currentBlock[1]
    nextBlocks = [[] for r in range(2)]
    if caseNum == 1:
        nextBlocks[0] = [x0 + 1, y0]
        nextBlocks[1] = [x0, y0 + 1]
    elif caseNum == 2:
        nextBlocks[0] = [x0 - 1, y0]
        nextBlocks[1] = [x0, y0 + 1]
    elif caseNum == 3:
        nextBlocks[0] = [x0 + 1, y0]
        nextBlocks[1] = [x0, y0 - 1]
    else:
        nextBlocks[0] = [x0 - 1, y0]
        nextBlocks[1] = [x0, y0 - 1]
    return nextBlocks