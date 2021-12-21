


class Image():
    def __init__(self, frame_no, kp, desc):
        self.frame_no = frame_no
        self.connections = []
        self.key_p = kp
        self.desc = desc
    
    def add_connection(self, con):
        self.connections.append(con)

        


class Scene_graph():
    def __init__(self, path_, nodes = None):
        self.path = path_
        self.frames = nodes

    def add_frame(self, frame):
        self.frames.append(frame)
    
    def get_frames(self):
        return self.frames
