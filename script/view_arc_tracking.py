import numpy as np
import matplotlib.pyplot as plt

def read_data_from_txt(fname: str, **readFlag):
    '''
    @brief: 从文本文件中读取数据，失败返回空 np.empty(0)
    @param: fname - 文件路径
    '''
    try:
        points = np.loadtxt(fname, **readFlag)
        # 将一行的数据转化为二维数组
        if (points.ndim == 1):
            points = np.reshape(points, (1, points.size))
    except IOError:
        print("File is not accessible: ", fname)
        points = np.empty(0)
    return points


def plot_fine(ax, name):
    # 缓冲区大小
    winSize = 4999
    
    pnt = read_data_from_txt(name+"raw_current.txt").transpose()
    fine = read_data_from_txt(name+"fine_current.txt").transpose()
    idx = read_data_from_txt(name+"index.txt").transpose()

    # 电流数据长度
    pntSize = int(pnt[0,0])

    # 时间跨度
    duration = []
    
    if (pntSize > winSize):
        beg = int(pntSize % winSize + 1)

        duration = np.linspace(pntSize-winSize+1, pntSize, winSize)
        
        tmp = pnt[beg:winSize+1, 0]
        pnt = np.append(tmp, pnt[1:beg, 0])
        
        tmp = fine[beg:winSize+1, 0]
        fine = np.append(tmp, fine[1:beg, 0])
    else:
        duration = np.linspace(1, pntSize, pntSize)
        pnt = pnt[1:int(pnt[0,0]+1), 0]
        fine = fine[1:int(fine[0,0]+1), 0]

    
    if (idx[0,0] > winSize):
        beg = int(idx[0,0] % winSize + 1)
        
        tmp = idx[beg:winSize+1, 0]
        idx = np.append(tmp, idx[1:beg, 0])
    else:
        idx = idx[1:int(idx[0,0]+1), 0]
    idx = [n for n in idx if (n>=duration[0] and n <=duration[-1])]
        
    print("duration: {}, {}".format(duration[0], duration[-1]))
    #print("pnt: ", pnt)
    #print("fine: ", fine)
    #print("idx", idx)

    # 相移
    fine = fine[5:]
    pnt = pnt[:-5]
    duration = duration[:-5]
    
    # 绘制电流曲线
    ax.plot(duration, pnt)
    ax.plot(duration, fine)

    # 绘制左右端点位置
    odd = idx[::2]
    even = idx[1::2]
    ax.scatter(odd, [fine[int(i-duration[0])] for i in odd], c='r')
    ax.scatter(even, [fine[int(i-duration[0])] for i in even], c='g')

    idxT = []
    aveA = []
    for ii in range(len(idx)-1):
        idxT.append(idx[ii])
        idxT.append(idx[ii+1])
        
        # 平均电流
        sumA = 0.0
        valid = 0
        for current in range(int(idx[ii]), int(idx[ii+1]-1)):
            tmpA = fine[int(current-duration[0])]
            if tmpA > sumA:
                sumA = tmpA
                valid = 1
            #if (tmpA > 50):
            #    sumA += tmpA
            #    valid += 1
        if (valid == 0):
            sumA = 0
        else:
            sumA /= valid
        print(int(idx[ii]), int(idx[ii+1]), sumA)
        #print(int(current-duration[0]), fine[int(current-duration[0])])
        aveA.append(sumA)
        aveA.append(sumA)
        
    #print(idxT)
    ax.plot(idxT, aveA)
    return


if __name__ == "__main__":
    fig = plt.figure('view window',figsize=(8,6))
    ax =plt.subplot(111)

    plot_fine(ax, "2/")

    plt.show()
