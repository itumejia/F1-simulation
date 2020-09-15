clc
clear all;

%gradas1
gx1=[495.8718,474.739,549.7572,570.8891,495.8718];
gy1=[2050.1827,2106.4595,2134.6285,2078.3517,2050.1827];
plot(gx1,gy1,'b')
hold on

%gradas2
gx2=[1322.7663,1401.458,1390.559,1311.866,1322.7663];
gy2=[750.4418,735.9309,676.822,691.332,750.4418];
plot(gx2,gy2,'b')


%graficar pista
format long 
aleatoriosX=[100 600 1300 1800];
aleatoriosY=[500 2000 800 2400];
pista=polyfit(aleatoriosX,aleatoriosY,3);
x=(100:0.1:1800); 
y=polyval(pista,x);
hold on
axis([0 3000 0 3000])
plot(x,y)

%Calcular la distancia entre todos los puntos de la pista y el origen
a=pista(1);
b=pista(2);
c=pista(3);
d=pista(4);
syms xpista
ecuacionpista=a*xpista^3+b*xpista^2+c*xpista+d;
LongitudesPista=[];
contador=1;
for j=100:10:1800
        derivada=diff(ecuacionpista,1);
        arg=(sqrt(1+(derivada)^2));
        Lon=int(arg,100,j);
        vpaLon=vpa(Lon);
        LongitudesPista(contador)=vpaLon;
        disp('Cargando...')
        %fprintf('%i\t%d\n',j,vpaLon)
        contador=contador+1;
end

%declaracion de variables
m = 805;  % masa del auto en Kg
Cd_1 = 0.9 ; % sin unidades comprobado Coeficiente de resistencia al aire (drag coeficient)
Ck = 0.8;%comprobado Coeficiente de friccion cinética
Cl = 1.88;%comprobado Coeficiente de sustitucion inversa
A = 0.05;%pendiente Area transversal m²
p = 0.78; % densidad del aire en kg/m^3 comprobada(checar pardo)
AF = 2.6; %area frontal del vehículo comproba
v_mov = 60; % velocidad del auto en m/s variable
v_inicial = 0.0; % Condición inicial de velocidad
syms v(t) 

%Potencias a lo largo del recorrido
Potencia1= 671130; %Primera potencia: Recta
% Potencia2= 0 ; %Segunda potencia: curva
% Potencia3= ; %Tercera potencia: recta
% Potencia4= ; %Cuarta potencia: curva
% Potencia5= ; %Quinta potencia: recta
division1=1800;
%division2=700;
%division3=1000;
%division4=1530;

%*******Calcular desplazamiento con primera potencia-----------------------
Fm=Potencia1/v_mov; % newtons
%Ecuación diferencial
Eqn_1=diff(v,t) == (Fm/m)-(Cd_1*p*AF/2/m)*v^2-(Ck*((m*9.8)+((Cl*A*p/2)*v^2)))/m;
% CONDICIÓN INICIAL DEL PROBLEMA PARA LA VELOCIDAD
Cond = v(0) == v_inicial;  % en m/s
% USO DEL COMANDO DSOLVE
velocidad=dsolve(Eqn_1,Cond);

%Se integra y deriva la función de velocidad para obtener funciones de
%aceleración y desplazamiento respecto al tiempo
aceleracion=diff(velocidad,1);
desplazamiento=int(velocidad);
%grafica=ezplot(velocidad,[0.0001,30])

syms s(t)%declarar variable para desplazamiento respecto al tiempo
syms a(t)%declarar variable para aceleración respecto al tiempo
fundesplazamiento=odeFunction(desplazamiento,s(t));%convierte la ecuacion de desplazamiento a una funcion que puede ser evaluada con matlab
funvelocidad=odeFunction(velocidad,v(t));%convierte la ecuacion de velocidad a una funcion que puede ser evaluada con matlab
funaceleracion=odeFunction(aceleracion,a(t));%convierte la ecuacion de aceleración a una funcion que puede ser evaluada con matlab


dx=[];%arreglo que va a guardar las coordenadas en x por cada segundo
dy=[];%arreglo que va a guardar las coordenadas en y por cada segundo
velocidad=[];%arreglo que va a guardar la velocidad por cada segundo
aceleracion=[];%arreglo que va a guardar la aceleración por cada segundo
perdidaenergia=[];%arreglo que va a guardar la perdida de energia acumulada por cada segundo
velocidadmaxima=[];%arreglo que va a guardar la velocidad máxima por cada segundo
perdidaenergiaanterior=0; %almacena la perdida de energ{ia en el segundo anterior
tiempo=1;
for i=0:150 %calcula todos los datos en cada segundo
    desplazamientoactual=feval(fundesplazamiento,i);%evaluar desplazamiento en cada segundo
    velocidadactual=feval(funvelocidad,i);%evaluar velocidad en cada segundo
    aceleracionactual=feval(funaceleracion,i);%evaluar aceleración en cada segundo
    vp=velocidadactual; %vv
    contador=1;
    for j=100:10:1800 %recorrido de la pista en x
        Y=polyval(pista,j); %evaluacion de los puntos x en y
        LonActual=LongitudesPista(contador); %guarda la long de curva en la que esta el contador
        contador=contador+1;
        if(LonActual >= desplazamientoactual)%compara las longitudes de curva con el desplazamiento calculado
            %fprintf('Desplazamiento\tLonCurva\n')
            %fprintf('%d\t%d\n',desplazamientoactual,LonActual)
            %coche=plot(j,Y,'or');
            dx(tiempo)=j; %se guardan las coordenadas en x que tendra en cada segundo
            dy(tiempo)=Y; %se gusradan las coordenadas en y que tendra en cada segundo
            velocidad(tiempo)=velocidadactual; %se gusradan la velocidad que tendra en cada segundo
            aceleracion(tiempo)=aceleracionactual;%se gusradan la aceleracion que tendra en cada segundo
            velocidadmaxima(tiempo)=sqrt((calcularRadio(j)*(4.47)*((m*9.81)+((Cl*A*p/2)*vp^2)))/m); %se gusradan la vel max que tendra en cada segundo
            perdidaenergia(tiempo)=perdidaenergiaanterior+(Cd_1*p*AF/2/m)*vp^2+(Ck*((m*9.8)+((Cl*A*p/2)*vp^2)))/m;%se gusrda la perdida de energia que tendra en cada segundo
            perdidaenergiaanterior=perdidaenergia(tiempo); %se guarda la perdida de energia que tuvo en ese segundo para sumarlo en el siguiente
            tiempo=tiempo+1;
            posicionfinalx=j;%posicion en x para ser usada mas tarde
            posicionfinaly=Y;%posicion en y para ser usada mas tarde
            velocidadfinal=velocidadactual; %velocidad en ese segundo para ser usada mas tarde
            break
        end
    end
    if posicionfinalx>=division1%revisar si la posicion en x alcanzo la posicion final de division1
        posicionfinalciclox=posicionfinalx; %almacena la posicion en la que se quedo anteriormente
        posicionfinalcicloy=posicionfinaly;
        break
    end
end




%Graficar todos los puntos de posición-------------------------------------

menu=1;

while menu==1
fprintf('Carga completada. Selecciona una opción para tu simulación:\n1. Programa sin derrape con potencia constante.\n2. Programa con derrape por exceso de velocidad.\n3. Salir.\n')
continuar=input('');

if continuar==1
for i=1:100%tiempo
    posicionactualx=dx(i);%la posicion en cada seg
    posicionactualy=dy(i);
    velocidadplot=velocidad(i);%vel en cada seg
    aceleracionplot=aceleracion(i);%aceleracion en cada seg
    perdidaenergiaplot=perdidaenergia(i);
    coche=plot(posicionactualx,posicionactualy,'or');%graficamos el punto con respecto al tiempo
    str=['Velocidad: ',num2str(velocidadplot),'m/s']; %texto letrero
    t= text(500,2000,str);%letrero
    str3=['Aceleracion: ',num2str(aceleracionplot),'m/s²'];%texto
    t3= text(500,1780,str3);%letrero
    str4=['Perdida de energia total: ',num2str(perdidaenergiaplot),'J'];
    t4= text(500,1200,str4);%letrero
    str2=['Tiempo total: ',num2str(i),'s'];
    t2= text(500,1580,str2);%letrero
    %disp(calcularRadio(j))
    if dx(i+1)>=1790%condicion para detener simulacion
        pause(5)%pausar 5 antes de borrar todo
        delete(coche)
        delete(t)
        delete(t2)
        delete(t3)
        delete(t4)
        break
    end
    pause(0.3)
    delete(coche)
    delete(t)
    delete(t2)
    delete(t3)
    delete(t4)
end

end

if continuar == 2
for i=1:100
    posicionactualx=dx(i);
    posicionactualy=dy(i);
    velocidadplot=velocidad(i);
    aceleracionplot=aceleracion(i);
    perdidaenergiaplot=perdidaenergia(i);
    coche=plot(posicionactualx,posicionactualy,'or');
    str=['Velocidad: ',num2str(velocidadplot),'m/s'];
    t= text(500,2000,str);
    str3=['Aceleracion: ',num2str(aceleracionplot),'m/s²'];
    t3= text(500,1780,str3);
    str4=['Perdida de energia total: ',num2str(perdidaenergiaplot),'J'];
    t4= text(500,1200,str4);
    str2=['Tiempo total: ',num2str(i),'s'];
    t2= text(500,1580,str2);
    %disp(calcularRadio(j))
    if velocidadmaxima(i)<velocidadplot%compual conparas la velocidad actual con la vel max
        derivadapendiente=sym2poly(derivada);%convertimos la derivada a una funcion que se puede evaluar
        pendiente=polyval(derivadapendiente,posicionactualx);%usamos polyval para encontrar la pendiente en el punto x actual
        b=(-pendiente*posicionactualx)+posicionactualy;%se utiliza la ecuacion punto pendiente para calcular b
        syms xt %nombramos variable simbolica
        tangente=pendiente*xt+b;%declaras la ecuacion de la recta tangente
        delete(coche)
        delete(t)
        delete(t2)
        delete(t3)
        delete(t4)
        str=['El auto se ha derrapado por exceso de velocidad'];
        t= text(500,2600,str);
        tangentee=sym2poly(tangente);%convertimos la tangente en algo que se pueda evaluar
        for j=posicionactualx:10:posicionactualx+200%se grafica la trayectoria de derrape por 200 metros a un paso 10
            plottangente=polyval(tangentee,j);%evaluas la tangente en el valor de j actual
            cochederrapado=plot(j,plottangente,'or');%se grafica el tramo de derrape
           pause(0.3)%lo pausamos un poco
           delete(cochederrapado)%lo borramos
        end
        delete(coche)
        delete(t)
        delete(t2)
        delete(t3)
        delete(t4)
        break
    end
    
    if dx(i+1)>=1790
        delete(coche)
        delete(t)
        delete(t2)
        delete(t3)
        delete(t4)
        break
    end
    pause(0.3)
    delete(coche)
    delete(t)
    delete(t2)
    delete(t3)
    delete(t4)
end
    
end

if continuar ==3
    break
end

end
%--------------------------------------------------------------------------
% Fm=0.1;
% syms v(t)
% Eqn_1=diff(v,t) == (Fm/m)-(Cd_1*p*AF/2/m)*v^2-(Ck*((m*9.8)+((Cl*A*p/2)*v^2)))/m;
% % CONDICIÓN INICIAL DEL PROBLEMA PARA LA VELOCIDAD
% Cond = v(0) == velocidadactual;  % en m/s
% % USO DEL COMANDO DSOLVE
% velocidad=dsolve(Eqn_1,Cond);
% hL_1=ezplot(velocidad,[0.0001,30])
% aceleracion=diff(velocidad,1);
% desplazamiento=int(velocidad);
% 
% syms s(t)
% fundesplazamiento=odeFunction(desplazamiento,s(t));%convierte la ecuacion de desplazamiento a una funcion que puede ser evaluada con matlab
% funvelocidad=odeFunction(velocidad,v(t));
% funaceleracion=odeFunction(aceleracion,a(t));
% 
% for i=0:22
%     desplazamientoactual=feval(fundesplazamiento,i);
%     velocidadactual=feval(funvelocidad,i);
%     aceleracionactual=feval(funaceleracion,i);
%     for j=posicionfinalx:10:1800
%         Y=0.375505602240919*(j)+1821.05;
%         Dis=sqrt(((j-posicionfinalx)^2)+((Y-posicionfinaly)^2));
%         if(Dis >= desplazamientoactual)
%             coche=plot(j,Y,'or');
%             str=['Velocidad: ',num2str(velocidadactual),'m/s'];
%             t= text(500,2000,str);
%             str2=['Tiempo total: ',num2str(22+i),'s'];
%             t2= text(500,1580,str2);
%             disp(calcularRadio(j))
%             pause(1) 
%             delete(coche)
%             delete(t)
%             delete(t2)
%             break
%         end
%     end
% 
% end
% 


