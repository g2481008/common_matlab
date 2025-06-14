function [] = waitPressEnterkey()
    kbhit('stop'); 
    kbhit('init');
    waitTime = 1.e-3;
    disp('Press Enterkey to execute loop.')
    while 1
        event = kbhit('event');
        key = get(event, 'KeyCode');
        pause(waitTime);
        if ~isempty(key) && key == 10
            break;
        end
    end
end