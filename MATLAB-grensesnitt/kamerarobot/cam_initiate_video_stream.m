function cam = cam_initiate_video_stream()
%identifiserer videostrøm
imaqhwinfo;
%DENNE METODEN BRUKES BARE NÅR WEBCAM BENYTTES
%Lager et videoobjekt som assosieres med webcameraet. Matlab velger her
%første og beste webcam som er tilgjengelig

cam = videoinput('winvideo');
