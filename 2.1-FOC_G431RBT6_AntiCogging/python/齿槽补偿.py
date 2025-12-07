import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline


def insert(sta, ed, iq_start, iq_end):
    step = end - sta + abs(ed - start)
    k = (iq_end - iq_start) / step
    insert_val = (end - sta) * k + iq_start
    return insert_val


data = np.genfromtxt(
    './FOC/vofa+.csv',
    delimiter=',',
    skip_header=1,  # 跳过标题行
    dtype=float,  # 或者 dtype=None 自动推断
    filling_values=0  # 缺失值填充为0
)

iq = data[:, 0]
rotor = data[:, 1]

start = -0.110159 * 7
end = 7 * (2 * np.pi - 0.110159)

num = 0
last = rotor[0]

index = list()
insert_val = list()

iq_list = list()
rotor_list = list()
iq_avg=0 #iq 平均值

rotor_matrix=list()
iq_matrix=list()
result=list()

scale=0.5  #resul 放缩系数

for i in range(len(rotor) - 1):
    if rotor[i + 1] - last < -40:
        num += 1
        index.append(i + 1)
        insert_val.append(insert(rotor[i], rotor[i + 1], iq[i], iq[i + 1]))
    last = rotor[i + 1]

num -= 1
print("总段数:",num)
print("插值处索引:",index)
print("插入值:",insert_val)

#分段 -->排序-->补值 
for i in range(num):
    iq_list.append(iq[index[i]:index[i + 1]])
    rotor_list.append(rotor[index[i]:index[i + 1]])
    idx = rotor_list[i].argsort()
    iq_list[i] = iq_list[i][idx]
    rotor_list[i] = rotor_list[i][idx]

    iq_list[i] = np.insert(iq_list[i], 0, insert_val[i])
    iq_list[i]=np.append(iq_list[i],insert_val[i+1])
    rotor_list[i]=np.insert(rotor_list[i],0,start)
    rotor_list[i]=np.append(rotor_list[i],end)



#踢除相等值
for j in range(num):
    last=rotor_list[j][0]
    repeat=list()
    for i in range(len(rotor_list[j])-1):
        if last == rotor_list[j][i+1]:
            repeat.append(i+1)
        last=rotor_list[j][i+1]
    rotor_list[j] = np.delete(rotor_list[j],repeat)
    iq_list[j] = np.delete(iq_list[j],repeat)

#减去平均值
iq_avg=np.mean(np.concatenate(iq_list))    
print(f"iq均值:{iq_avg}")
for i in range(num):
    iq_list[i]=iq_list[i]-iq_avg
    



#将多次循环的插值转化为 二位arry
for i in range(num):
    print(f"第{i+1}段插值")
    cs = CubicSpline(rotor_list[i], iq_list[i])
    x_new=np.linspace(start, end, 1024)
    rotor_matrix.append(x_new)
    iq_matrix.append(cs(x_new))

rotor_matrix = np.vstack(rotor_matrix)
iq_matrix = np.vstack(iq_matrix)

#求结果-->以 [r1,r1...]形式输出到文本
result =scale* np.mean(iq_matrix, axis=0)

with open("./FOC/result.txt", "w", encoding="utf-8") as f:
    f.write(str(result.tolist()))


#绘图
plt.figure()
plt.subplot(3, 1, 1) 
for i in range(num):
    plt.plot(rotor_matrix[i], iq_matrix[i], label=f"{i + 1}")
plt.title("insertion")
# plt.xlabel("rotor")
plt.ylabel("iq")
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 2) 
for i in range(num):
    plt.plot(rotor_list[i],  iq_list[i], label=f"{i + 1}")
plt.title("origional")
# plt.xlabel("rotor")
plt.ylabel("iq")
plt.legend()
plt.grid(True)


plt.subplot(3, 1, 3) 
plt.plot(rotor_matrix[0],  result, label="result")
plt.title("result")
plt.xlabel("rotor")
plt.ylabel("iq")
plt.legend()
plt.grid(True)


plt.show()
