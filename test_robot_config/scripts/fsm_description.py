#!/usr/bin/env python3

class FSM(object):
    def __init__(self):
        
        self.current_state = 'init'
            #init
            #search
            #aiming
            #moving
            #stop
        
        self.is_goal_visible  = False
        self.is_camera_stares = False   # камера смотрит прямо на цель
        self.is_camera_placed = False   # камера повёрнута на 0 градусов
        # два флага выше истинны <=> тело направлено точно на цель
        self.is_body_directed = False   # тело направлено почти на цель
        self.is_goal_reached  = False
    
    def switch_state(self):
        if self.current_state == 'init':
            if self.is_goal_reached:
                self.current_state = 'stop'
                return
            if self.is_goal_visible:
                self.current_state = 'aiming'
                return
            else:
                self.current_state = 'search'
                return
            
        elif self.current_state == 'search':
            if self.is_goal_visible:
                self.current_state = 'aiming'
                return
            
        elif self.current_state == 'aiming':
            if not self.is_goal_visible:
                self.current_state = 'search'
                return
            if self.is_body_directed:
                self.current_state = 'steering'
                return
            if self.is_goal_reached:
                self.current_state = 'stop'
                return
            return
        
        elif self.current_state == 'steering':
            if not self.is_goal_visible:
                self.current_state = 'search'
                return
            if not self.is_body_directed:
                self.current_state = 'aiming'
                return
            if self.is_camera_stares and self.is_camera_placed:
                self.current_state = 'moving'
                return
            if self.is_goal_reached:
                self.current_state = 'stop'
                return
            return
        
        elif self.current_state == 'moving':
            if not self.is_goal_visible:
                self.current_state = 'search'
                return
            if  (not self.is_camera_stares) or (not self.is_camera_placed):
                self.current_state = 'steering'
                return
            if self.is_goal_reached:
                self.current_state = 'stop'
            return 
        #'stop'
                
    
#if __name__ == '__main__':
    
