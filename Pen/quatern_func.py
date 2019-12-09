import numpy as np

def quaternProd(a, b):
        # lm=q(1); p1=q(2); p2=q(3); p3=q(4);
        # q1=[lm -p1 -p2 -p3
        #     p1  lm -p3  p2
        #     p2  p3  lm -p1
        #     p3 -p2  p1  lm]*m;
        lm = a[0]
        p1 = a[1]
        p2 = a[2]
        p3 = a[3]
        temp = np.array([[lm, -p1, -p2, -p3],
                         [p1,  lm, -p3,  p2],
                         [p2,  p3,  lm, -p1],
                         [p3, -p2,  p1,  lm]])
        # print temp, b
        q1 = np.dot(temp,np.transpose([b]))
        q1 = np.transpose(q1)
        q1 = q1[0]
        return q1

def quaternInv(a):
    b = [a[0], -a[1], -a[2], -a[3]]

    return b

def quaternRot(Vec,Quat):
    V = [0., Vec[0],Vec[1],Vec[2]]
    # print V
    Ve = quaternProd(quaternProd(quaternInv(Quat),V),Quat)
    Ve = np.array([[Ve[1],Ve[2],Ve[3]]])
    return Ve