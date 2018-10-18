function save_figure_to_img(figure_num, title_name, saved_img_name, high_reso)
% save figure to file. saved_img_name doesn't contain suffix.
% if want to save larger image, could set  fig = figure(figure_num);fig.Position=[100 220 1127 840]; before calling this function

	if (nargin<4)
		high_reso = false;
	end
	if (high_reso)
		output_mode = '-dpng';
	else
		output_mode = '-djpe';
	end
    fig2=figure(figure_num);title(title_name); %fig2.Position=[133 646 570 422];
    fig = gcf;fig.PaperPositionMode = 'auto'; % for save image the the same as figure size
    print(saved_img_name,output_mode,'-r0');  % djpe for jpg    dpng for png   add sufix automatically
end