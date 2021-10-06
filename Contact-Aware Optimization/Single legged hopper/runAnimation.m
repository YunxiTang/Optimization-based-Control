function runAnimation(t_hist, x_hist)
    %% Create video of system
    Nt = length(t_hist);
    h = t_hist(2) - t_hist(1);
    fps = 20;
    % Turn off plot visibility temporarily during frame rendering
    set(0,'DefaultFigureVisible','off');

    % Create figure for frames
    fig = figure(100);
    ax = gca;
    ax.NextPlot = 'ReplaceChildren';
    
    % Compute number of frames
    num_frame = length(1:ceil((1.0 ./ fps) ./ h):Nt);

    % Structure to store the frames
    frames(num_frame) = struct('cdata',[],'colormap',[]);

    % Plot limits
    xl = [];
    yl = [];

    % Counter for frames processed
    n = 1;

    % Open video for writing
    vid = VideoWriter('Animation_v2', 'Uncompressed AVI');
    vid.FrameRate = fps;
    open(vid);
    
    % Loop over points in time to process frames 
    for k = 1:ceil((1.0 ./ fps) ./ h): Nt 
        %------------------Add plots-----------------%
        plot([x_hist(1,k) x_hist(3,k)], [x_hist(2,k) x_hist(4,k)], 'k-', 'LineWidth', 3.0);
        hold on;
        plot(x_hist(1,k), x_hist(2,k), 'ro', 'MarkerSize',20, 'MarkerFaceColor','r');
        hold on;
        plot(x_hist(3,k), x_hist(4,k), 'bo', 'MarkerSize',10, 'MarkerFaceColor','b');
        hold on;
        plot(-1.2:0.1:1.2, 0.* ones(1, length(-1.2:0.1:1.2)),'Color', 'k','LineWidth',2.0);
        hold off;
        %-----------------End plots-----------------%
        % Obtain or apply plot limits for consistency across frames
        if k == 1
            xlim([-1.5 1.5]);
            ylim([0.0 4.0]);
            axis equal;
            xl = xlim();
            yl = ylim() ;
            ylim(yl);
        else
            xlim(xl);
            ylim(yl);
        end

        % Apply blank background
%         axis off;

        % Render the frame
        drawnow;

        % Record the frame and store in the structure
        frames(n) = getframe(fig);

        % Write frame to video
        writeVideo(vid, frames(n));

        % Display progress to command line
        fprintf("Rendering video frame %d out of %d...\n", n, num_frame);

        % Increment frame counter
        n = n + 1;
    end

    % Close the video
    close(vid);

    % Renable plot visibility to view movie and other plots
    set(0,'DefaultFigureVisible','on')

    % Create fresh figure object for the movie
    figure;

    % Render the movie and set to loop for a very large number of times
    fprintf("done rendering video frames, presenting video...\n")
    movie(gcf, frames, 9999, fps);
end