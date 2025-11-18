function [note_sequence, time_intervals] = GetTwinkleTwinkleScore()
    % GetTwinkleTwinkleScore - Define the musical score for "Twinkle Twinkle Little Star"
    %
    % Output:
    %   note_sequence - Cell array of note names in playing order
    %   time_intervals - Array of time intervals between notes (seconds)
    %
    % Musical structure:
    %   Twinkle Twinkle Little Star melody in Do-Re-Mi notation
    %   Each note is held for a specific duration
    
    % Define the melody (note names)
    % "Twinkle Twinkle Little Star" in solfège notation
    % Do Do Sol Sol La La Sol - Fa Fa Mi Mi Re Re Do
    % Sol Sol Fa Fa Mi Mi Re - Sol Sol Fa Fa Mi Mi Re
    % Do Do Sol Sol La La Sol - Fa Fa Mi Mi Re Re Do
    
    note_sequence = {
        % First line: "Twinkle twinkle little star"
        'Do', 'Do', 'Sol', 'Sol', 'La', 'La', 'Sol', ...
        % Second line: "How I wonder what you are"
        'Fa', 'Fa', 'Mi', 'Mi', 'Re', 'Re', 'Do', ...
        % Third line: "Up above the world so high"
        'Sol', 'Sol', 'Fa', 'Fa', 'Mi', 'Mi', 'Re', ...
        % Fourth line: "Like a diamond in the sky"
        'Sol', 'Sol', 'Fa', 'Fa', 'Mi', 'Mi', 'Re', ...
        % Fifth line: "Twinkle twinkle little star"
        'Do', 'Do', 'Sol', 'Sol', 'La', 'La', 'Sol', ...
        % Sixth line: "How I wonder what you are"
        'Fa', 'Fa', 'Mi', 'Mi', 'Re', 'Re', 'Do'
    };
    
    % Define time intervals (seconds)
    % Quarter note = 0.5s, half note = 1.0s
    % Most notes are quarter notes, some are half notes
    
    num_notes = length(note_sequence);
    time_intervals = zeros(1, num_notes);
    
    % Define rhythm pattern
    % 0.5s for quarter notes, 1.0s for half notes (at end of phrases)
    rhythm_pattern = [
        % Line 1: Do Do Sol Sol La La Sol(half)
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0, ...
        % Line 2: Fa Fa Mi Mi Re Re Do(half)
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0, ...
        % Line 3: Sol Sol Fa Fa Mi Mi Re(half)
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0, ...
        % Line 4: Sol Sol Fa Fa Mi Mi Re(half)
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0, ...
        % Line 5: Do Do Sol Sol La La Sol(half)
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0, ...
        % Line 6: Fa Fa Mi Mi Re Re Do(half)
        0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0
    ];
    
    time_intervals = rhythm_pattern(1:num_notes);
    
    fprintf('Musical score loaded: %d notes\n', num_notes);
    fprintf('Total duration: %.2f seconds\n', sum(time_intervals));
end
