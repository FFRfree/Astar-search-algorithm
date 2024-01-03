# %%
import matplotlib.pyplot as plt
from copy import deepcopy
# %%
map = []
with open(r'A_start_search_algothrim\Astar_search\maze.txt','rb') as f:
	for lines in f:
		map.append(list(lines.decode('utf-8').strip()))
# map = [[' ',' ',' ',' ',' ',' ',' ',' ',' ',' '],
#        [' ',' ',' ',' ',' ',' ',' ',' ',' ',' '],
#        [' ',' ',' ',' ',' ',' ',' ',' ',' ',' '],
#        [' ',' ',' ',' ',' ',' ',' ',' ',' ',' '],
#        [' ',' ',' ',' ',' ',' ',' ',' ',' ',' '],
#        [' ',' ',' ',' ',' ',' ',' ',' ',' ',' '],
#        [' ',' ',' ',' ',' ',' ',' ',' ',' ',' '],
#        [' ',' ',' ',' ',' ',' ',' ',' ',' ',' '],
#        [' ',' ',' ',' ',' ',' ',' ',' ',' ',' '],
#        [' ',' ',' ',' ',' ',' ',' ',' ',' ',' ']]
# map = [[' ','#',' ',' ',' ',' ',' ',' ',' ',' ',' '],
#        [' ','#',' ',' ','#',' ','#',' ',' ',' ',' '],
#        [' ','#',' ',' ','#',' ',' ',' ',' ',' ',' '],
#        [' ',' ',' ',' ','#',' ',' ',' ',' ',' ',' '],
#        [' ','#',' ',' ',' ',' ','#',' ',' ',' ',' '],
#        [' ','#',' ',' ','#',' ','#',' ',' ',' ',' '],
#        [' ','#',' ',' ','#',' ',' ',' ',' ',' ',' '],
#        [' ',' ',' ',' ','#',' ','#',' ','#','#',' '],
#        [' ','#',' ',' ','#',' ','#',' ',' ',' ',' '],
#        [' ','#',' ',' ',' ',' ','#',' ',' ',' ',' ']]

max_row = len(map)
max_col = len(map[0])
# startp  = (0,0)
# goalp = (9,9)
startp = (1,1)
goalp = (3,39)
path = []
closed = []
open = []


# %%
class Node():
    def __init__(self,pos,parent = None):
        self.pos = pos
        # self.predict_c= abs(goalp[1]-pos[1])+abs(goalp[0]-pos[0])
        self.predict_c = (pow(goalp[1]-pos[1],2)+pow(goalp[0]-pos[0],2))**0.5
        if isinstance(parent, int):
            self.already_c= parent
        elif isinstance(parent, Node):
            self.already_c = parent.already_c + 1
            self.ppos = parent.pos
        else:
            raise Exception('Parent must be a int or a node!')
        self.total_cost = self.already_c + self.predict_c
    def __eq__(self,other):
        return self.pos == other.pos
    def __hash__(self):
        return 20*self.pos[0] + 1*self.pos[1] #hash
    def __lt__(self,other):
        return self.total_cost < other.total_cost
    def __repr__(self):
        return f"d_{self.pos[0]}_{self.pos[1]}"

#%%    
def legal_step_or_not(i):
    if i[0]<0 or i[1]<0 or i[0] >max_col or i[1]>max_row:
        return False
    elif map[i[1]][i[0]] == '#':
        return False
    else:
        return True

def color_print():
    mmap = deepcopy(map)
    for r in range(max_row):
        for c in range(max_col):
            if mmap[r][c] == ' ':
                mmap[r][c] = (255,255,255)
            elif mmap[r][c] == '#':
                mmap[r][c] = (0,0,0)
    for i in open:
        mmap[i.pos[1]][i.pos[0]] = (0,255,0)
    for i in closed:
        mmap[i.pos[1]][i.pos[0]] = (237,145,33)
    mmap[startp[1]][startp[0]] = (160,32,240)
    mmap[goalp[1]][goalp[0]] = (160,32,240)
    plt.imshow(mmap)
    plt.ion()
    plt.pause(0.1)
    plt.clf()

def printpathimg():
    mmap = deepcopy(map)
    for r in range(max_row):
        for c in range(max_col):
            if mmap[r][c] == ' ':
                mmap[r][c] = (255,255,255)
            elif mmap[r][c] == '#':
                mmap[r][c] = (0,0,0)
    mmap[startp[1]][startp[0]] = (160,32,240)
    mmap[goalp[1]][goalp[0]] = (160,32,240)
    for i in path:
        mmap[i[1]][i[0]] = (255,255,0)
    plt.ioff()
    plt.imshow(mmap)
    plt.show()

def backtrace(traceback):
    path.append(traceback)
    temp = Node(traceback,parent=256)
    a = closed.pop(closed.index(temp)).ppos
    del temp #去实例化
    if a != startp:
        backtrace(a)

def expand_frontier(parentNode):
    if parentNode.pos == goalp:
        print('find the goal point')
        
        return parentNode.ppos

    x = parentNode.pos[0]
    y = parentNode.pos[1]

    temp = [(x-1,y),(x,y-1),(x+1,y),(x,y+1),(x-1,y-1),(x+1,y+1),(x-1,y+1),(x+1,y-1)]
    for i in temp:
        if not legal_step_or_not(i):
            #是否撞墙或出界
            continue
        else:
            exec(f'd_{i[0]}_{i[1]}=Node({i},parent= parentNode )') #实例化
            temp_d = locals()[f'd_{i[0]}_{i[1]}']

            if temp_d in closed: #判断是否走到走过的路上，我们要保证closed集中路径永远花费最低
                index1 = closed.index(temp_d)
                if closed[index1].total_cost > temp_d.total_cost: #走过的路绕路了，
                    open.append(temp_d) #将新路径放入open集合中,以便日后更新从此点出发的路径，此时append和insert区别不大
                    closed.pop(index1) #从closed中移除
                    continue
                else:#新路径成本更大
                    exec(f"del d_{i[0]}_{i[1]}") #去实例化
            elif temp_d in open: #是否已经在frontier中了
                index2 = open.index(temp_d)
                if open[index2].total_cost > temp_d.total_cost: #原来的路径花费更大
                    open.pop(index2)
                    open.append(temp_d)
                    continue
                else:
                    exec(f"del d_{i[0]}_{i[1]}") 
            else:
                open.insert(0,temp_d) #下一次遍历open集就可以再花费相同的情况下优先走走过的路径。
    print('expand the frontier')
    return 'finished, push parentNode to closed.'
    
def next_p(threshold = 2,lastone = 'starting Node'):
    if lastone == 'starting Node':
        return open.pop(0)
    cost_close = [] #花费接近的Node
    least_cost = open[0].total_cost
    for node in open:
        if node.total_cost <= least_cost +threshold:
            cost_close.append(node)
    # if lastone in cost_close
    return #Node obj

def astar_search():
    sp = Node(startp,parent=0)
    open.append(sp)
    while open != []:
        color_print() #打印每一步的图像
        print(open)
        open.sort()
        lesscost_p = next_p() #阈值问题：不能遇到困难就放弃 (unfinished)
        print(lesscost_p)
        traceback = expand_frontier(lesscost_p)
        if  traceback == 'finished, push parentNode to closed.':
            closed.append(lesscost_p)
        else: #find the goal
            backtrace(traceback)
            print('The final path:\n')
            print(path)
            printpathimg()
            break


if __name__ == "__main__":
    color_print()
    astar_search()    
    
# %%
