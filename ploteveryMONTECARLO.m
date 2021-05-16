function  ploteveryMONTECARLO(PARAMETRI_SIMULATI,PARAMETRI_SIMULATI_SUCCESS,PARAMETRI_SIMULATI_FAIL)
close all;
aaa = [7 8 9 17 18 19 20 21 23 24 26 27 28 29];
for i = 1:14
    figure(i);
    histogram(PARAMETRI_SIMULATI(aaa(i),:),50);grid on,hold on;
	histogram(PARAMETRI_SIMULATI_FAIL(aaa(i),:),50);grid on,hold on;
    histogram(PARAMETRI_SIMULATI_SUCCESS(aaa(i),:),50);legend('PARAMETRI','FAIL','SUCCESS')
end


