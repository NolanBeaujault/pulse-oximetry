// Permet de communiquer avec notre carte Arduino via le port COM
s = serialport("COM24", 115200);
configureTerminator(s,"LF");

// permet de plot les données reçues
figure(1);
line = plot(nan, nan); % Ligne initiale vide 
xlabel('Temps');
ylabel('Valeur des données');
grid on;

% On initialise le temps pour le graphique
startTime = tic;
x_data = [];
y_data = [];

while true
    % On lit les données envoyées par la carte Arduino
    raw_data = readline(s);
    disp(raw_data);
    % On vérifie si la donnée est valide
    val = str2double(raw_data);


        % Mise à jour des données 
        t = toc(startTime);
        x_data(end+1) = t;
        y_data(end+1) = val;

        % Mise à jour du graphique
        set(line, 'XData', x_data, 'YData', y_data);
        xlim([max(0, t-20) t]); % Afficher seulement les 20 derniers instants
        drawnow;
end