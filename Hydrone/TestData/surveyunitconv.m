clc
clear
close all

survey=readtable("Test_17\Test17_survey.txt");
outline=readtable("Test_17\Test17_outline.txt");
path=readtable("Test_17\Test17_path.txt");


survey.RawDepth1=survey.RawDepth1./3.2808399;
survey.RawDepth2=survey.RawDepth2./3.2808399;
survey.Corr_Depth1=survey.Corr_Depth1./3.2808399;
survey.Corr_Depth2=survey.Corr_Depth2./3.2808399;
survey.DOL=survey.DOL./3.2808399;
survey.DBL=survey.DBL./3.2808399;
survey.GPSElevation=survey.GPSElevation./3.2808399;
survey.Speed=survey.Speed./1.94384449;

outline.RawDepth1=outline.RawDepth1./3.2808399;
outline.RawDepth2=outline.RawDepth2./3.2808399;
outline.Corr_Depth1=outline.Corr_Depth1./3.2808399;
outline.Corr_Depth2=outline.Corr_Depth2./3.2808399;
outline.DOL=outline.DOL./3.2808399;
outline.DBL=outline.DBL./3.2808399;
outline.GPSElevation=outline.GPSElevation./3.2808399;
outline.Speed=outline.Speed./1.94384449;


writetable(survey,'surveyB1')
writetable(outline,'outlineB1')
writetable(path,'pathB1')