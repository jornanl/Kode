function cam = cam_initiate_video_stream()
%identifiserer videostr�m
imaqhwinfo;
%DENNE METODEN BRUKES BARE N�R WEBCAM BENYTTES
%Lager et videoobjekt som assosieres med webcameraet. Matlab velger her
%f�rste og beste webcam som er tilgjengelig

cam = videoinput('winvideo');
