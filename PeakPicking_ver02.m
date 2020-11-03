%%Eren GÜL PhD thesis 3DOF building model simulation and determining modal
%parameters from acceleration data from each floor
bCol=0.1;     %damping coefficient of columns
colM=1e-4;  %graphical representation of columns with negligble weight
%%Running the 3 DOF building model
%% 

sim('ndof_4_col_w_damper',10.000)


%%Getting acceleration data from each floor
len=size(first_floor_acc.data,1); %data length that is evaluated
x=zeros(len,3); %sensor data matrix

x(:,1)=first_floor_acc.data(1:len,1);   %first floor data loaded
x(:,2)=x(:,1)+second_floor_acc.data(1:len,1);  %second floor data loaded
x(:,3)=x(:,2)+third_floor_acc.data(1:len,1);   %third floor data loaded

%%Fast Fourier Transform of the acceleration data
Fs=200;
NFFT = 2^nextpow2(len); % Next power of 2 from length of y
Xf = fft(x,NFFT,1)/len;
f = Fs/2*linspace(0,1,NFFT/2+1);


pks=zeros(3);
locs=zeros(3);


for i=1:3
    %% 
    %%Plotting Fourier transform of the acceleration data in order
    %%Finding peaks and peak locations of Fourier Transformed data
    %figure(i)
    figure(i)
    plot(f*2*pi,2*abs(Xf(1:NFFT/2+1,i))) 
    title(strcat('Amplitude spectrum of the', {' '}, int2str(i), '. floor acceleration'))
    xlabel('Frequency (rad/s)')
    ylabel(strcat('|acc_', int2str(i),'(w)|'))
    axis([0 160 0 5.2])
    
    
    [pks(:,i),locs(:,i)] = findpeaks(2*abs(Xf(1:NFFT/2+1,i)),'MinPeakDistance',50);
end
pks

%%
%Plotting Input Force Magnitude Spectrum
q=inForce.data(1:len,1);
Qf = fft(q,NFFT,1)/len;

figure(4)
plot(f*2*pi,2*abs(Qf(1:NFFT/2+1,1))) 
title(strcat('Amplitude Spectrum of Input Force'))
xlabel('Frequency (rad/s)')
ylabel(strcat('Force Magnitude [N]'))
axis([0 160 0 0.2])

%%
%%Obtaining mode vectors by finding oscillation directions of the floors at
%each level at each peak at each locations
for i=1:3
    ang_x=angle([Xf(locs(i,1),1) Xf(locs(i,1),2) Xf(locs(i,1),3)])*180/pi;
    sign_x=[1 1 1];
    
        if (abs(ang_x(1)-ang_x(2))>40 && abs(ang_x(1)-ang_x(2))<320)
            sign_x(2)=-sign_x(2);
        end
        if (abs(ang_x(1)-ang_x(3))>40 && abs(ang_x(1)-ang_x(3))<320)
            sign_x(3)=-sign_x(3);
        end
        sign_x
    pks(i,:)=pks(i,:).*sign_x;
end

V=[pks(:,1) pks(:,2) pks(:,3)];  %Acceleration vectors at each mode frequency
vn=[norm(V(1,:)); norm(V(2,:)); norm(V(3,:))];              %Norms of Mode Vectors
modeVectors=[V(1,:)/vn(1); V(2,:)/vn(2); V(3,:)/vn(3)];     %Mode Vectors

fprintf(strcat('Mode frequencies in order in Hz are :',mat2str(f(locs(:,1))),'\n'));
fprintf(strcat('Mode frequencies in order in rad/s are :',mat2str(f(locs(:,1))*2*pi),'\n'));

fprintf(strcat('Mode vector for Mode 1 :',mat2str(modeVectors(1,:)),'\n'));
fprintf(strcat('Mode vector for Mode 2 :',mat2str(modeVectors(2,:)),'\n'));
fprintf(strcat('Mode vector for Mode 3 :',mat2str(modeVectors(3,:)),'\n'));