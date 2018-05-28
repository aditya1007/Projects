import numpy as np
import time

#start_time = time.clock()
#main()
#print time.clock() - start_time,'seconds'

def find_SP(graph, start, end, p=[]):
                p = p + [start]
                if start == end:
                    return p
                if not graph.has_key(start):
                    return None
                shortestpath = None
                for node in graph[start]:
                         
                    if node not in p:
                        newpath = find_SP(graph, node, end, p)
                        if newpath:
                            if not shortestpath or len(newpath) < len(shortestpath):
                                shortestpath = newpath
                return shortestpath

def find_Shortest_Path_Pixels (ImgSeg, V, p, q, PathType):

        start_time = time.clock()
        #main()
        
        
        '''
        MODE =0

A = [[3,1,2,1],
     [2,2,0,2],
     [1,2,1,1],
     [1,0,1,2]]
     #[1,1,1,1]]
V=[1,2]
P=[3,0]
Q=[0,3]

ImgSeg= [[3,4,5,3],
     [1,3,2,3],
     [3,2,1,4],
     [1,3,1,1]]

ImgSeg= [[10,20,30,40,50],
     [20,30,20,20,50],
     [50,10,40,50,40],
     [40,30,10,10,40],
     [50,10,20,30,30]]

ImgSeg= [[4,5,10,16,64,5],
     [5,16,5,5,10,10],
     [16,5,64,10,4,5],
     [10,10,4,16,4,5],
     [5,16,4,20,64,10],
     [5,10,16,25,16,4]]

'''
        MODE = PathType
        A = ImgSeg

        P=p
        Q=q

        px=P[0]
        py=P[1]
        qx=Q[0]
        qy=Q[1]


        a= np.array(A)
        v=np.array(V)

        I = a.copy()
        x=a.shape[0]
        y=a.shape[1]

        i=0
        k=1
        while i<x:
                j=0
                while j<y:
                        I[i][j]=k
                        k=k+1
                        j=j+1
                i=i+1

        #print'a',A;
        #print 'I',I;

        source = I[px][py]
        destn = I[qx][qy]

        #print 's',source;
        #print 'd',destn;


        i=0
        #print'x',x;
        #print'y',y;
        Graph={'':[]}
        while i<x:
                j=0
                while j<y:
                        graph = {I[i][j]:[]}
                        d1dpr=0
                        d1cpr=0
                        d2cpr=0
                        d2dpr=0
                        d3apr=0
                        d3bpr=0
                        d4apr=0
                        d4dpr=0
                        
                        if a[i][j] in v:
                                #print'i',i
                                #print'j',j

                                if (i<x-1):
                                        
                                        if a[i+1][j] in v:      #A
                                                
                                                graph[I[i][j]].append(I[i+1][j])        #[I[i+1][j]]}#,I[i][j+1],I[i-1][j],I[i][j-1]}
                                                d3apr = 0
                                                d4apr = 0
                                        elif a[i+1][j] not in v:
                                                d3apr = 1
                                                d4apr = 1
                                if (j<y-1):
                                        if a[i][j+1] in v:      #B
                                                graph[I[i][j]].append(I[i][j+1]) 
                                                d2bpr = 0
                                                d3bpr = 0
                                        elif a[i][j+1] not in v:
                                                d2bpr = 1
                                                d3bpr = 1
                                                
                                if (i>0):
                                        
                                        if a[i-1][j] in v:      #C
                                                graph[I[i][j]].append(I[i-1][j]) 
                                                d1cpr = 0
                                                d2cpr = 0
                                        elif a[i-1][j] not in v:
                                                d1cpr = 1
                                                d2cpr = 1

                                if (j>0):               
                                        if a[i][j-1] in v:      #D
                                                graph[I[i][j]].append(I[i][j-1])
                                                d1dpr = 0
                                                d4dpr = 0
                                        elif a[i][j-1] not in v:
                                                d1dpr = 1
                                                d4dpr = 1
                                              
                                if ((MODE==2)&(d1dpr ==1)&(d1cpr ==1))or(MODE==1):
                                        
                                        if (i>0)&(j>0):                                    
                                                if a[i-1][j-1] in v:                                               
                                                        
                                                        graph[I[i][j]].append(I[i-1][j-1])
                                                        
                                                        
                                if ((MODE==2)&(d2cpr ==1)&(d2bpr ==1))or(MODE==1):
                                        
                                        if (i>0)&(j<y-1):               
                                                if a[i-1][j+1] in v:
                                                        graph[I[i][j]].append(I[i-1][j+1])
                                
                                if ((MODE==2)&(d3apr ==1)&(d3bpr ==1))or(MODE==1):
                                        
                                        if (i<x-1)&(j<y-1):               
                                                if a[i+1][j+1] in v:
                                                        graph[I[i][j]].append(I[i+1][j+1])
                                                        
                                if ((MODE==2)&(d4apr ==1)&(d4dpr ==1))or(MODE==1):
                                        
                                        if (i<x-1)&(j>0):               
                                                if a[i+1][j-1] in v:
                                                        graph[I[i][j]].append(I[i+1][j-1])
                                     
                        Graph.update(graph)
                        
                        j=j+1
                i=i+1

        del  Graph['']                                                      

        graph = Graph
        #print'graph',graph;

        

        R=find_SP(graph, source, destn)
        
        if(R == None):
                print "Path does not exist"
        else:
                

                k=0
                l=[]
                
                r=np.array(R)
                z=r.shape[0]
                #print'z',z;
                SPath=[]
                while k<z:
                        i=0
                        while i<x:
                                j=0
                                while j<y:
                                        #print'i',i
                                        #print'j',j
                                        #print'k',k
                                        if R[k]==I[i][j]:
                                                l=[i,j]
                                                SPath.append(l)
                                                
                                        j=j+1
                                i=i+1
                                #print'j',j
                        k=k+1
                        
                #print'R',r;
                #print'Shortest Path:',SPath    ;                            
                sp=np.array(SPath)
                Length = ((sp.shape[0])-1)
                #print'R',Length;
                #print time.clock() - start_time,'seconds'
                TIME = time.clock() - start_time
                #return SPath
        if(R == None):
                return None
        else:
                return SPath, Length, TIME
        
                
        
