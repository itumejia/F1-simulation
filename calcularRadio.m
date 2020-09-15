function Radio=calcularRadio(x)
format long %obtener resultados con muchos decimales
syms x10 %declarar x10 como una variable
%x1 y y1 son arreglos de 4 puntos en nuestra trayectoria que se utilizan en polyfit
%para obtener nuestra función
x1=[100 600 1300 1800];
y1=[500 2000 800 2400];
n=3;%en n indicamos el grado deseado de nuestra función
curva=polyfit(x1,y1,n);%curva va a tener los coeficientes de nuestra función polinomial de grado 3
%Guardamos cada coeficiente en una variable independiente
a=curva(1);
b=curva(2);
c=curva(3);
d=curva(4);
%Armamos la ecuación con los coeficientes
ecuacion=a*x10^3+b*x10^2+c*x10+d;
%Derivamos la ecuación y la guardamos en la variable derivada
derivada=diff(ecuacion,1);
%Derivamos la ecuación dos veces y la guardamos en la variable segunda derivada
segundaderivada=diff(ecuacion,2);
%sym2poly pasa los coeficientes de las ecuaciones derivada y
%segundaderivada a arreglos
arreglo1=sym2poly(derivada);
arreglo2=sym2poly(segundaderivada);
%polyval evalúa el polinomio con el parámetro x otorgado
pdeval=polyval(arreglo1,x);
sdeval=polyval(arreglo2,x);
%Calcula el radio en la coordenada x indicada
Radio=((1+pdeval^2)^(3/2))/abs(sdeval);
end