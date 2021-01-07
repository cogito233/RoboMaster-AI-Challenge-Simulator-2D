import numpy as np
class kenal(object):
    def __init__(self, car_num, render=False, record=True):
        self.car_num = car_num
        self.render = render
        # below are params that can be challenged depended on situation
        self.bullet_speed = 12.5
        self.motion = 6
        self.rotate_motion = 4
        self.yaw_motion = 1
        self.camera_angle = 75 / 2
        self.lidar_angle = 120 / 2
        self.move_discount = 0.6
        # above are params that can be challenged depended on situation
        self.map_length = 800
        self.map_width = 500
        self.theta = np.rad2deg(np.arctan(45/60))
        self.record=record
        self.areas = np.array([[[580.0, 680.0, 275.0, 375.0],
                                [350.0, 450.0, 0.0, 100.0],
                                [700.0, 800.0, 400.0, 500.0],
                                [0.0, 100.0, 400.0, 500.0]],
                               [[120.0, 220.0, 125.0, 225.0],
                                [350.0, 450.0, 400.0, 500.0],
                                [0.0, 100.0, 0.0, 100.0],
                                [700.0, 800.0, 0.0, 100.0]]], dtype='float32')
        self.barriers = np.array([[350.0, 450.0, 237.5, 262.5],
                                  [120.0, 220.0, 100.0, 125.0],
                                  [580.0, 680.0, 375.0, 400.0],
                                  [140.0, 165.0, 260.0, 360.0],
                                  [635.0, 660.0, 140.0, 240.0],
                                  [325.0, 350.0, 400.0, 500.0],
                                  [450.0, 475.0, 0.0, 100.0]], dtype='float32')
        if render:
            global pygame
            import pygame
            pygame.init()
            self.screen = pygame.display.set_mode((self.map_length, self.map_width))
            pygame.display.set_caption('RM AI Challenge Simulator')
            self.gray = (180, 180, 180)
            self.red = (190, 20, 20)
            self.blue = (10, 125, 181)
            # load barriers imgs
            self.barriers_img = []
            self.barriers_rect = []
            for i in range(self.barriers.shape[0]):
                self.barriers_img.append(pygame.image.load('./imgs/barrier_{}.png'.format('horizontal' if i < 3 else 'vertical')))
                self.barriers_rect.append(self.barriers_img[-1].get_rect())
                self.barriers_rect[-1].center = [self.barriers[i][0:2].mean(), self.barriers[i][2:4].mean()]
            # load areas imgs
            self.areas_img = []
            self.areas_rect = []
            for oi, o in enumerate(['red', 'blue']):
                for ti, t in enumerate(['bonus', 'supply', 'start', 'start']):
                    self.areas_img.append(pygame.image.load('./imgs/area_{}_{}.png'.format(t, o)))
                    self.areas_rect.append(self.areas_img[-1].get_rect())
                    self.areas_rect[-1].center = [self.areas[oi, ti][0:2].mean(), self.areas[oi, ti][2:4].mean()]
            # load supply head imgs
            self.head_img = [pygame.image.load('./imgs/area_head_{}.png'.format(i)) for i in ['red', 'blue']]
            self.head_rect = [self.head_img[i].get_rect() for i in range(len(self.head_img))]
            self.head_rect[0].center = [self.areas[0, 1][0:2].mean(), self.areas[0, 1][2:4].mean()]
            self.head_rect[1].center = [self.areas[1, 1][0:2].mean(), self.areas[1, 1][2:4].mean()]
            self.chassis_img = pygame.image.load('./imgs/chassis_g.png')
            self.gimbal_img = pygame.image.load('./imgs/gimbal_g.png')
            self.bullet_img = pygame.image.load('./imgs/bullet_s.png')
            self.info_bar_img = pygame.image.load('./imgs/info_bar.png')
            self.bullet_rect = self.bullet_img.get_rect()
            self.info_bar_rect = self.info_bar_img.get_rect()
            self.info_bar_rect.center = [200, self.map_width/2]
            pygame.font.init()
            self.font = pygame.font.SysFont('info', 20)
            self.clock = pygame.time.Clock()

            self.sp_map = np.zero([map_width, map_length], dtype='uint32')
            self.sp_value = np.zero([map_width, map_length], dtype='float')
            self.sp_flag = np.zero([map_width, map_length], dtype='uint32')
            self.sp_ff = np.zero([map_width, map_length], dtype='uint32')
            self.sp_route=np.zero([map_width*map_length*5,2],dtype='uint32')
            self.sp_last=np.zero([map_width, map_length],dtype='uint32')



    def sp_init(self,car_th):
        def sp_init_change_value(f):
            global r,self.sp_map
            for i in range(math.floor(f[0]),math.ceil(f[1])):
                for j in range(math.floor(f[2]),math.ceil(f[3])):
                    if (self.sp_map[i][j]==0):
                        self.sp_map[i][j]=1
                        r+=1
                        seq[r]=[i,j]
        self.sp_map=np.zero([map_width,map_length],dtype='uint32')
        self.sp_value=np.zero([map_width,map_length],dtype='float')
        self.sp_flag=np.zero([map_width,map_length],dtype='uint16')
        seq=np.zero([map_width*map_length*5,2],dtype='uint32')

        l,r,c=0,0,[(0,1),(0,-1),(1,0),(-1,0)]
        for i in range(barriers.shape[0]):sp_init_change_value(barrrs[i])
        for i in range(cars.shape[0]):
            if (i!=car_th):
                sp_init_change_value(np.array([cars[i][0]-30,cars[i][0]+30,cars[i][1]-30,cars[i][1]+30]))

        while (l<r):
            l+=1
            for dx,dy in c:
                x,y=dx+seq[sp_l][0],dy+seq[l][1]
                if (self.sp_flag[x][y]==0):
                    self.sp_flag[x][y]=1
                    self.sp_value[x][y]=self.sp_value[[seq[l][0]][seq[l][1]]+1
                    r+=1
                    seq[r]=[x,y]

        for i in range(1,map_width+1):
            for j in range(1,map_length+1):
                if (self.sp_value[i][j]<=25):
                    self.sp_map[i][j]=1
                else:self.sp_value[i][j]-=25;
        self.sp_value=200/(self.sp_value+1e-5)+1

    def sp_calc(self,p_begin):
        self.sp_map = np.zero([map_width, map_length], dtype='uint32')
        self.sp_value = np.zero([map_width, map_length], dtype='float')
        self.sp_flag = np.zero([map_width, map_length], dtype='uint32')
        self.sp_ff = np.zero([map_width, map_length], dtype='uint32')
        self.sp_last = np.zero([map_width, map_length,2], dtype='uint32')
        seq=np.zero([mawidth*map_length*5,2],dtype='uint32')
        f=np.zero([map_width*map_length*5,2],dtype='float')
        for i in range(f.shape[0]):
            for j in range(f.shape[1]):f[i][j]=1e15

        x,y=p_begin
        l,r,c=0,1,[(0,1),(0,-1),(1,0),(-1,0)]
        seq[r]=[x,y]
        f[x]=0
        while (l<r&&r<map_width*map_length*5-4):
            l+=1
            for dx,dy in c:
                x,y=seq[l][0]+dx,seq[l][1]+dy
                if (self.sp_map[x][y] == 0 && f[x][y] + 1e-5 > f[seq[l][0]][seq[l][1]] + self.sp_value[x][y]):
                    f[x][y] = f[seq[l][0]][seq[l][1]] + self.sp_value[x][y]
                    self.sp_last[x][y]=seq[l]
                    self.sp_ff[x][y]=1
                    if (self.sp_flag[x][y]==0):
                        self.sp_flag[x][u]=1
                        r+=1
                        seq[r]=[x,y]
            self.sp_flag[seq[l][0]][seq[l][1]]=0

    def sp_follow_the_road(self,p_begin,p_end):
        self.sp_calc(p_begin)
        z,d=[],0
        def dfs(x,y):
            global z
            if (self.sp_last[i][j][1]!=0):dfs(last[x][y][0],last[x][y][1])
            d+=1
            z[d]=(x,y)
        if (ff[p_end[0]][p_end[1]]==0):return "No such path"
        dfs(p_end[0],p_end[1])
        x,y,xx,yy,now_x,now_y=0,0,[0]*map_width*map_length,[0]*map_width*map_length,z[1][0],z[1][1]
        zz=[(now_x,now_y)]
        for i in range(2,d+1):
            if (z[i][0]-z[i-1][0]!=0):
                x+=z[i][0]-z[i-1][0]
                if (y!=0):
                    now_x,now_y=now_x+x,now_y+y
                    zz+=[(now_x,now_y)]
                    x,y=0,0
            if (z[i][1]-z[i-1][1]!=0):
                y+=z[i][1]-z[i-1][1]
                if (x!=0):
                    now_x,now_y=now_x+x,now_y+y
                    zz+=[(now_x,now_y)]
                    x,y=0,0
        if (x!=0||y!=0)
            now_x,now_y=now_x+x,now_y+y
            zz+=[(now_x,now_y)]
            x,y=0,0
        for x,y in zz:print(x,y)
        sp_route=np.array(zz)
        return zz
