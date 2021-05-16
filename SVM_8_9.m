target_point_all=[target_point_8_SUCCESS;target_point_7_SUCCESS];
target_point_all2=[target_point_8_FAIL;target_point_7_FAIL];
target_point_all=double([target_point_all,target_point_all2]');
Y=double([linspace(1,1,length(target_point_8_SUCCESS)),linspace(0,0,length(target_point_8_FAIL))]');
close all;
% figure;
% subplot(1,2,1);
% plot(target_point_all(1:length(target_point_8_SUCCESS),1),target_point_all(1:length(target_point_8_SUCCESS),2),'m+');hold on;
% plot(target_point_all(length(target_point_8_SUCCESS):end,1),target_point_all(length(target_point_8_SUCCESS):end,2),'c*');
% subplot(1,2,2);
% model = svmtrain(Y(1:400),target_point_all(1:400,:),'-s 0 -c 10 -t 1 -g 1 -r 1 -d 3 showplot');
% [prelabel,accuracy,decision_values]=svmpredict(Y(401:500),target_point_all(401:500,:),model); 
% C = svmclassify(model,target_point_all(401:500,:),'showplot',true);
trt = 0.8;
target_point_all_train=target_point_all(1:trt*length(Y),:);
Y_train=Y(1:trt*length(Y),:);
target_point_all_test=target_point_all(trt*length(Y)+1:end,:);
Y_test=Y(trt*length(Y)+1:end,:);

SVMModel=fitcsvm(target_point_all_train,Y_train,'KernelFunction','rbf','OptimizeHyperparameters',{'BoxConstraint','KernelScale'}, 'HyperparameterOptimizationOptions',struct('ShowPlots',true));
dd=0.02;
[x1Grid,x2Grid]=meshgrid(min(target_point_all(:,1)):dd:max(target_point_all(:,1)),min(target_point_all(:,2)):dd:max(target_point_all(:,2)));
xGrid = [x1Grid(:),x2Grid(:)];
[~,scores] = predict(SVMModel,xGrid);
sv = SVMModel.SupportVectors;
% figure
% gscatter(target_point_all(:,1),target_point_all(:,2),Y)
% hold on
% plot(sv(:,1),sv(:,2),'ko','MarkerSize',10);
% % predict(SVMModel,target_point_all(401:end,:));
figure;
h(1:2) = gscatter(target_point_all(:,1),target_point_all(:,2),Y,'rb','.');
hold on
h(3) = plot(sv(:,1),sv(:,2),'ko');
contour(x1Grid,x2Grid,reshape(scores(:,2),size(x1Grid)),[0 0],'k');