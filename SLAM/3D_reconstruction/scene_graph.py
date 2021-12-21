


class Image():
    def __init__(self, frame_no, kp, desc):
        self.frame_no = frame_no
        self.connections = []
        self.key_p = kp
        self.desc = desc
        self.connection_indexes = []
    
    def add_connection(self, con):
        self.connections.append(con)
        self.connection_indexes.append(con.frame_no)
    def get_connection_indexes(self):
        return self.connection_indexes

        


class Scene_graph():
    def __init__(self, path_):
        self.path = path_
        self.frames = []

    def add_frame(self, frame):
        self.frames.append(frame)
    
    def add_connection(self, frame1, frame2):
        index1 = frame1.frame_no
        index2 = frame2.frame_no
        if index1 not in self.frames[index2-1].connection_indexes and index2 not in self.frames[index1-1].connection_indexes:
            # print('index1: ', index1)
            # print('index2: ', index2)
            # print()
            self.frames[index1-1].add_connection(frame2)
            self.frames[index2-1].add_connection(frame1)
        else:
            print(' --- CONNECTION ALREADY ESTABLISHED --- ')
        
    def print_graph(self):
        for node in self.frames:
            print('frame ' + str(node.frame_no))
            print('connections: ')
            # print(type(node.connections[0]))
            for neighbour in node.connections:
                print(neighbour.frame_no)


    def get_frames(self):
        return self.frames
    
    def get_frame(self, frame_no):
        return self.frames[frame_no-1]
