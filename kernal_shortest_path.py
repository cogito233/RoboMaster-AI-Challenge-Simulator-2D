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

    def sp_init_change_value(self,f):
        for i in range(math.floor(f[0]),math.ceil(f[1])):
            for j in range(math.floor(f[2]),math.ceil(f[3])):
                if (sp_map[i][j]==0):
                    sp_map[i][j]=1

    def sp_init(self,car_th):
        sp_map=np.zero([map_width,map_length],dtype='uint32')
        sp_value=np.zero([map_width,map_length],dtype='float')
        sp_flag=np.zero([map_width,map_length],dtype='uint16')
        sp_seq=np.zero([map_width*map_length*5,2],dtype='uint32')
        sp_l,sp_r,sp_c=0,0,[[0,1],[0,-1],[1,0],[-1,0]]
        for i in range(barriers.shape[0]):sp_init_change_value(barrrs[i])
        for i in range(cars.shape[0]):
            if (i!=car_th):
                sp_init_change_value(np.array([cars[i][0]-30,cars[i][0]+30,cars[i][1]-30,cars[i][1]+30]))

        while (sp_l<sp_r):
            sp_l+=1
            for dx,dy in sp_c:
                x,y=dx+sp_seq[sp_l][0],dy+sp_seq[sp_l][1]
                if (flag[x][y]==0):
                    flag[x][y]=1
                    value[x][y]=value[sp_seq[sp_l][0]][sp_seq[sp_l][1]]+1
                    sp_r+=1
                    seq[sp_r]=[x,y]


    def sp_calc(self,p_begin,p_end):
        x,y=p_begin
        xx,yy=p_end
        sp_flag=np.zero([map_width,map_length],dtype='uint16')

    def sp_follow_the_road(self,p_begin,p_end):
        def dfs():
            if (last[i][j])

