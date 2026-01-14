clearvars
clear all
close all
clc


zed = ZED2i();
zed.rConnect();

intr = zed.rGetIntrinsics();
disp(intr);

zed.rDisconnect();
