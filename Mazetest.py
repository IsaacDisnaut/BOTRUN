import time
from collections import deque
import random
import paho.mqtt.client as mqtt

compare=[]
rot=0
i=0
unique_pos_turn=[]
path_events = [] 
tarx=1 #targetx
tary=19 #targety

def solve_maze(maze, start, end):
    rows, cols = len(maze), len(maze[0])
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
    queue = deque([(*start, [start])])  # (row, col, path)

    while queue:
        x, y, path = queue.popleft()

        if (x, y) == end:
            return path

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols and maze[nx][ny] in ['🔴', 'H', 'B']:
                maze[nx][ny] = '😄' # Mark as visited
                queue.append((nx, ny, path + [(nx, ny)]))

    return None

commands = deque()  # queue เก็บคำสั่ง
reply =""
status=""
xx=0
yy=0

def on_connect(client, userdata, flags, rc):
    print("✅ Connected to broker")
    client.subscribe("state")
    client.subscribe("target")  # รับข้อมูลจาก topic target


def on_message(client, userdata, msg):
    global status, xx, yy, facing

    payload = msg.payload.decode()
    topic = msg.topic
    print(f"📩 Received from {topic}: {payload}")

    # --- ถ้า topic เป็น "state" ---
    if topic == "state":
        commands.append(payload)

    # --- ถ้า topic เป็น "target" ---
    elif topic == "target":
        parts = payload.split("/")
        if len(parts) == 4:
            try:
                status = parts[0]        # 'final'
                xx = int(parts[1])      # ค่าตำแหน่ง x
                yy = int(parts[2])      # ค่าตำแหน่ง y
                facing = int(parts[3])  # มุมหุ่นยนต์
                print(f"🎯 Target update → state={status}, xx={xx}, yy={yy}, facing={facing}")
            except ValueError:
                print("⚠️ Invalid number in payload:", payload)
        else:
            print("⚠️ Invalid format for 'target' topic. Expected: state/xx/yy/facing")

        if status == "final":
            tarx=xx
            tary = yy
        print(f"your target is : {tarx},{tary}")
   
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("broker.hivemq.com", port=1883, keepalive=60)
client.loop_start()

rows=21
col=21
rowop=rows
colop=col
direction=""
maze = [['⚫' for _ in range(col)] for _ in range(rows)]
x=14 #row
y=12 #column
startx=x
starty=y
path=[]
maze[x][y]="🟢"
maze[tarx][tary]="⚫"
ii=0
ii_return=0
step=2
ready_to_solve=0
time.sleep(1)
for row in maze:
        print(row)
while(True):
    if status == "final":
        tarx=20-xx*2
        tary=yy*2
        print("Target:",tarx,tary)
        status=''
    # direction=input("Here:")
    if commands:
        direction = commands.popleft()  # เอาคำสั่งแรกออกจาก queue
    else:
        time.sleep(0.1)
        continue

    if direction == "r" and y < rowop:
        
        for _ in range(step):
            if maze[x][y] != "🟢":
                maze[x][y] = '🔴'
            y += 1
        maze[x][y] = "😄"

        maze = [list(row) for row in zip(*maze)][::-1]
        dummy=x
        x=col-1-y
        y=dummy

        dummy=tarx
        tarx=col-1-tary
        tary=dummy

        rot=(rot+90)%360
        for row in maze:
            print(row)

    if direction == "b" and x < rowop:
        if maze[x][y] != "🟢":
            maze[x][y] = '🔴'
        x += 1
        maze[x][y] = '🔴'
        x += 1
        maze[x][y] = "😄"
        maze = [list(row) for row in zip(*maze)][::-1]
        dummy=x
        x=col-1-y
        y=dummy

        dummy=tarx
        tarx=col-1-tary
        tary=dummy
        rot=(rot-90)%360
        maze = [list(row) for row in zip(*maze)][::-1]
        dummy=x
        x=col-1-y
        y=dummy

        dummy=tarx
        tarx=col-1-tary
        tary=dummy
        rot=(rot-90)%360


        for row in maze:
            print(row)

    if direction == "l" and y > 1:
        
        for _ in range(step):
            if maze[x][y] != "🟢":
                maze[x][y] = '🔴'
            y -= 1
        maze[x][y] = "😄"
        maze = [list(row[::-1]) for row in zip(*maze)]
        dummy=y
        y=rowop-1-x
        x=dummy

        dummy=tary
        tary=rowop-1-tarx
        tarx=dummy
        rot=(rot-90)%360
        for row in maze:
            print(row)
    
    if direction == "s" and x > 2:
        
        for _ in range(step):
            if maze[x][y] != "🟢":
                maze[x][y] = '🔴'
            x -= 1
        maze[x][y] = "😄"
        for row in maze:
            print(row)

    if direction == "stop":
        # if rot == 270:
        #     dummy=x
        #     x=col-1-y
        #     y=dummy
        #     maze = [list(row) for row in zip(*maze)][::-1]
        #     rot=0
        # elif rot == 90:
        #     dummy=y
        #     y=rowop-1-x
        #     x=dummy
        #     maze = [list(row[::-1]) for row in zip(*maze)]
        #     rot=0
        # elif rot == 180:
        #     dummy=x
        #     x=col-1-y
        #     y=dummy
        #     dummy=x
        #     x=col-1-y
        #     y=dummy
        #     maze = [list(row[::-1]) for row in zip(*maze)]
        #     maze = [list(row[::-1]) for row in zip(*maze)]
        #     rot=0
        # elif abs(rot)==360:
        #     print("Noth")
        tarx=x
        tary=y
        for row in maze:
            print(row)
        print("Start",startx,starty)
        print("your target is here")
        print(tarx,tary)
        
        
    if direction == "solve":
        if rot == 270 :
            dummy=x
            x=col-1-y
            y=dummy

            dummy=tarx
            tarx=col-1-tary
            tary=dummy

            maze = [list(row) for row in zip(*maze)][::-1]
            rot=0
        elif rot == 90 :
            dummy=y
            y=rowop-1-x
            x=dummy

            dummy=tary
            tary=rowop-1-tarx
            tarx=dummy

            maze = [list(row[::-1]) for row in zip(*maze)]
            rot=0
        elif abs(rot) == 180:
            dummy=x
            x=col-1-y
            y=dummy

            dummy=tarx
            tarx=col-1-tary
            tary=dummy

            dummy=x
            x=col-1-y
            y=dummy

            dummy=tarx
            tarx=col-1-tary
            tary=dummy

            maze = [list(row[::-1]) for row in zip(*maze)]
            maze = [list(row[::-1]) for row in zip(*maze)]
            rot=0
        elif abs(rot)==360:
            print("Noth")
        for row in maze:
            print(row)
            
        start = (startx,starty)
        end = (tarx,tary)
        maze[x][y] = "🔴"
        path = solve_maze([row[:] for row in maze], start, end)
        print(path)
        path_matrix = []
        for k in range(len(maze)):
            row = []
            for j in range(len(maze[0])):
                if (k,j) in path and maze[k][j] in ['🔴','😄','H','B','🟢']:
                    row.append('😄')  # แทนทางเดินด้วย emoji หน้ายิ้ม
                else:
                    row.append(maze[k][j])
            path_matrix.append(row)

        for row in path_matrix:
            print(' '.join(row))

        # 🔹 หาจุดเลี้ยว
        def direction_change(a, b):
            dx = b[0]-a[0]
            dy = b[1]-a[1]
            return (dx, dy)

        rows = len(path_matrix)
        cols = len(path_matrix[0])
        dirs = [(-1,0),(1,0),(0,-1),(0,1)]

        marked_matrix = [row[:] for row in path_matrix]

        # ----------------------------
        # 🔹 สร้าง array เก็บจุดเลี้ยวและจุดแยก
        # ----------------------------
        path_events = []  # [(ตำแหน่ง, 'เลี้ยว'/'แยก', ทิศทาง)]

        for idx, (x,y) in enumerate(path):
            # ตรวจสอบจุดเลี้ยว
            if 0 < idx < len(path)-1:
                d1 = direction_change(path[idx-1], path[idx])
                d2 = direction_change(path[idx], path[idx+1])
                if d1 != d2:
                    cross = d1[0]*d2[1]-d1[1]*d2[0]
                    if cross > 0:
                        turn = "left"
                        marked_matrix[x][y]='🤡'
                    elif cross < 0:
                        turn = "right"
                        marked_matrix[x][y]='🤡'
                    else:
                        turn = "forward"
                        marked_matrix[x][y]='🤡'
                    path_events.append(((x,y),'เลี้ยว',turn))
            #if path_events[-1][0]
           
            # ตรวจสอบจุดแยก
            neighbors=[]
            for dx,dy in dirs:
                ni,nj = x+dx, y+dy
                if 0<=ni<rows and 0<=nj<cols:
                    if path_matrix[ni][nj] in ['😄','🔴']:
                        neighbors.append((ni,nj))
            if len(neighbors) >= 3:
                marked_matrix[x][y]='🤡'
                # หาทิศทางที่ path เดินต่อ
                if idx < len(path)-1:
                    d = direction_change(path[idx], path[idx+1])
                    cross = 0
                    # ใช้วิธีเดียวกับจุดเลี้ยว
                    if idx > 0:
                        d_prev = direction_change(path[idx-1], path[idx])
                        cross = d_prev[0]*d[1]-d_prev[1]*d[0]
                    if cross > 0:
                        turn = "left"
                    elif cross < 0:
                        turn = "right"
                    else:
                        turn = "forward"
                else:
                    turn = "forward"
                path_events.append(((x,y),'แยก',turn))
        print("This")
        print(path_events)

        # ----------------------------
        # 🔹 แสดงผล
        # ----------------------------
        print("Grid แสดงจุดเลี้ยวและจุดแยก:")
       
        maze_summary_str = '\n'.join([' '.join(row) for row in marked_matrix])

# แสดงผลเหมือน print เดิม
        print(maze_summary_str)

        print("\nArray สรุปทางตามเส้นทาง:", path_events)
        unique_pos_turn = []  # list เก็บผลลัพธ์
        seen_pos = set()      # set ใช้ตรวจสอบ pos ที่ซ้ำ

        for pos, kind, turn in path_events:  # path_events จากโค้ดก่อนหน้า
            if pos not in seen_pos:
                unique_pos_turn.append(turn)
                seen_pos.add(pos)    
                    
    # แสดงผล
        print("Array pos และ turn แบบไม่ซ้ำ pos:")
        print(pos)
        if pos == (tarx,tary):
            print("Nooo")
            unique_pos_turn.pop()
            print(unique_pos_turn)
        unique_pos_turn.append('')
        print(unique_pos_turn)
        

    if direction == "return":
        unique_pos_turn.reverse()  # กลับลิสต์

        for i, d in enumerate(unique_pos_turn):
            if d == 'left':
                 unique_pos_turn[i] = 'right'
            elif d == 'right':
                 unique_pos_turn[i] = 'left'
            print(d)
        print(unique_pos_turn)
    print(x, y, "rot:", rot)
    print(x,y)
    

    if direction == "ask":
        dir=unique_pos_turn[ii]
        client.publish("dir",dir)
        print(dir)
        ii=ii+1

    if direction == "ask2":
        dir=unique_pos_turn[ii_return]
        client.publish("dir",dir)
        ii_return=ii_return+1
        print(dir)
    
    if abs(rot) == 360:
        rot = 0
    direction = ''
    