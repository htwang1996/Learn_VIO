clear 
close all


lambda = dlmread('lambda.txt');
dt = 1:length(lambda);
figure
plot(dt, lambda , '-x');
grid on;                           
hold on; 

my_lambda = dlmread('mylambda.txt');

dt = 1:length(my_lambda);

plot(dt, my_lambda , '-o');

title('阻尼因子迭代变化曲线图')

xlabel('迭代次数');                
ylabel('lambda_对应值');             
legend('贺博的更新策略','我的更新策略');      
                          
