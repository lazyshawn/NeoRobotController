import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

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


def plot_fine(ax):
    pnt = read_data_from_txt("interp_pos.txt", delimiter=',').transpose()
    ax.plot(pnt[0], pnt[1])
    ax.set_aspect(1)
    print(pnt)
    pass


if __name__ == "__main__":
    fig = plt.figure('view window',figsize=(10,5))
    # 定义2x2网格
    gs = GridSpec(2,2)
    ax = fig.add_subplot(gs[:,0])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax1 = fig.add_subplot(gs[0,1])
    ax2 = fig.add_subplot(gs[1,1])

    plot_fine(ax)
    
    plt.show()
