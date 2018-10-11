function first = playAudio( first, clip )

%Plays the designated audio clip
switch clip
    case 'intro'
        [y,Fs] = audioread('Intro.wav');
        %disp('played Intro')
    case 'yellow'
        if first == 1
            [y,Fs] = audioread('First_Air.wav');
            first = 0;
            %disp('played First_Air')
        else
            [y,Fs] = audioread('Air.wav');
            %disp('played Air')
        end
    case 'green'
        [y,Fs] = audioread('Earth.wav');
        %disp('played Earth')
    case 'blue'
        [y,Fs] = audioread('Water.wav');
        %disp('played Water')
        
end

sound(y,Fs);

end

