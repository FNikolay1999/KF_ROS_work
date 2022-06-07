import numpy as np
import numpy.random

# Моделирование данных датчика
def simulateSensor(samplesCount, noiseSigma, dt):
   # Шум с нормальным распределением. мат. ожидание = 0, среднеквадратичное отклонение = noiseSigma
   noise = numpy.random.normal(loc = 0.0, scale = noiseSigma, size = samplesCount)

   trajectory = np.zeros((3, samplesCount)) # Так будем знать, как со временем менялось то или иное значение. Записаны в виде массива из значений (pos, velo, accel). Другими, словами, матрица

   position = 0
   velocity = 1.0
   acceleration = 0.0

   for i in range(1, samplesCount):
       position = position + velocity * dt + (acceleration * dt ** 2) / 2.0 #Расчёт по формулам. Моделирование включает в себя ускорение
       velocity = velocity + acceleration * dt
       acceleration = acceleration

       trajectory[0][i] = position
       trajectory[1][i] = velocity
       trajectory[2][i] = acceleration

   measurement = trajectory[0] + noise #Добавляем к позициям шум

   return trajectory, measurement # Истинное значение и данные "датчика" с шумом