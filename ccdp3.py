#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Robótica Computacional - 
# Grado en Ingeniería Informática (Cuarto)

import math
import sys
from math import *
import numpy as np
import matplotlib.pyplot as plt
import colorsys as cs

# ******************************************************************************
# Declaración de funciones

def muestra_origenes(O,final=0):
  # Muestra los orígenes de coordenadas para cada articulación
  print('Origenes de coordenadas:')
  for i in range(len(O)):
    print('(O'+str(i)+')0\t= '+str([round(j,3) for j in O[i]]))
  if final:
    print('E.Final = '+str([round(j,3) for j in final]))

def muestra_robot(O,obj):
  # Muestra el robot graficamente
  plt.figure()
  plt.xlim(-L,L)
  plt.ylim(-L,L)
  T = [np.array(o).T.tolist() for o in O]
  for i in range(len(T)):
    plt.plot(T[i][0], T[i][1], '-o', color=cs.hsv_to_rgb(i/float(len(T)),1,1))
  plt.plot(obj[0], obj[1], '*')
  plt.pause(0.0001)
  plt.show()
  
#  input()
  plt.close()

def matriz_T(d,th,a,al):
   
  return [[cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th)]
         ,[sin(th),  cos(th)*cos(al), -sin(al)*cos(th), a*sin(th)]
         ,[      0,          sin(al),          cos(al),         d]
         ,[      0,                0,                0,         1]
         ]

def cin_dir(th,a):
  #Sea 'th' el vector de thetas
  #Sea 'a'  el vector de longitudes
  T = np.identity(4)
  o = [[0,0]]
  for i in range(len(th)):
    T = np.dot(T,matriz_T(0,th[i],a[i],0))
    tmp=np.dot(T,[0,0,0,1])
    o.append([tmp[0],tmp[1]])
  return o

# ******************************************************************************
# Cálculo de la cinemática inversa de forma iterativa por el método CCD

# valores articulares arbitrarios para la cinemática directa inicial
th=[0.,0.,0.] 
a =[5.,5.,5.]
ty=["rev","rev","des"]
L = sum(a) # variable para representación gráfica
# modificar para que siempre se vea el robot
EPSILON = .8
MAX_ANGLE = math.pi / 2    # Límite superior (90 grados)
MIN_ANGLE = -(math.pi / 2) # Límite superior (90 grados)
MAX_LENGTH = 15
MIN_LENGTH = 0
values = [[ math.pi  , MAX_ANGLE       ,MAX_ANGLE ],
          [-math.pi  , MIN_ANGLE       ,MIN_ANGLE ],
          [MAX_LENGTH, MAX_LENGTH      ,MAX_LENGTH],
          [MIN_LENGTH, MIN_LENGTH      ,MIN_LENGTH]]

#plt.ion() # modo interactivo

# introducción del punto para la cinemática inversa
if len(sys.argv) != 4:
  sys.exit("python " + sys.argv[0] + " x y valor de tabla")
objetivo=[float(i) for i in sys.argv[1:3]]
valor_tabla=sys.argv[3]
O=cin_dir(th,a)
#O=zeros(len(th)+1) # Reservamos estructura en memoria
 # Calculamos la posicion inicial
print ("- Posicion inicial:")
muestra_origenes(O)

dist = float("inf")
dist = np.linalg.norm(np.subtract(objetivo,O[-1][-1]))
prev = 0.
iteracion = 1
L += (dist / float(valor_tabla))
print("DISTANCIA: " + str(dist))
while (dist > EPSILON and abs(prev-dist) > EPSILON/100.):
  prev = dist
  O=[cin_dir(th,a)]
  # Para cada combinación de articulaciones:
  for i in range(len(th)):
    # cálculo de la cinemática inversa: comprobar desde aqui 
    # En clase se recomendó que usaramos la trigonometría
        #Pseudocódigo:
    # obtener coordenadas de p
    p = O[-1][len(th) - i - 1]
    # obtener coordenadas de t
    t = objetivo
    # obtener coordanadas de EF.
    EF = O[-1][-1]
    if ty[len(th) - i - 1] == "rev":

      alpha2  = math.atan2(t[1]-p[1],t[0]-p[0])
      alpha1 = math.atan2(EF[1]-p[1],EF[0]-p[0])
      
      th[len(th) - i -1 ] = th[len(th) - i -1] + (alpha2 - alpha1)
      th[len(th) - i - 1] = ((th[len(th) - i - 1] + math.pi) % (2 * math.pi)) - math.pi
      #th[(len(th)-1)-i] = th[(len(th)-1)-i] + math.remainder(incThetha, tau) ?
      
      # Restringir el ángulo al límite superior
      if th[len(th) - i - 1] > values[0][len(th) - i - 1]:
          th[len(th) - i - 1] = values[0][len(th) - i - 1]

      # Restringir el ángulo al límite superior
      if th[len(th) - i - 1] < values[1][len(th) - i - 1]:
          th[len(th) - i - 1] = values[1][len(th) - i - 1]
          
    if ty[len(th) - i - 1] == "des":
      #como sacar w:
      #w=sumatorio(thethai) desde i=1 hasta i actual
      #suma de todos los angulos de las articulaciones 
      w = 0
      w = sum(th[:len(th) - i - 1]) #?
      #u(cos(w),sen(w))
      u = ( math.cos(w), math.sin(w))
      
      #v(xt-xef,yt-yef)
      v = (t[0]-EF[0],t[1]-EF[1])
      d = np.dot(u,v)
      a[len(a) - i - 1] = a[len(a) - i - 1] + d
      
      # Restringir la longitud al límite superior
      if a[len(a) - i - 1] > values[2][len(th) - i - 1]:
          a[len(a) - i - 1] = values[2][len(th) - i - 1]

      # Restringir la longitud al límite superior
      if a[len(a) - i - 1] < values[3][len(th) - i - 1]:
          a[len(a) - i - 1] = values[3][len(th) - i - 1]

    O.append(cin_dir(th,a))


  dist = np.linalg.norm(np.subtract(objetivo,O[-1][-1]))
  print ("\n- Iteracion " + str(iteracion) + ':')
  muestra_origenes(O[-1])
  muestra_robot(O,objetivo)
  print ("Distancia al objetivo = " + str(round(dist,5)))
  iteracion+=1
  O[0]=O[-1]

if dist <= EPSILON:
  print ("\n" + str(iteracion) + " iteraciones para converger.")
else:
  print ("\nNo hay convergencia tras " + str(iteracion) + " iteraciones.")
print ("- Umbral de convergencia epsilon: " + str(EPSILON))
print ("- Distancia al objetivo:          " + str(round(dist,5)))
print ("- Valores finales de las articulaciones:")
for i in range(len(th)):
  print ("  theta" + str(i+1) + " = " + str(round(th[i],3)))
for i in range(len(th)):
  print ("  L" + str(i+1) + "     = " + str(round(a[i],3)))
