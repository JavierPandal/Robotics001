import math

radio = float(input("Introduce el radio de la circunferencia: "))
perimetro = 2 * math.pi * radio
area = math.pi * radio ** 2

print(f"El perímetro de la circunferencia es {perimetro}")
print(f"El área de la circunferencia es {area}")
print(f"radio: {radio:.2f}")


volumen = (4/3) * math.pi * radio ** 3
print(f"El volumen de la esfera es {volumen}")