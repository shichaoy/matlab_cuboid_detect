function colors = get_id_color(i)
object_colors={'red','green','blue','cyan','magenta','black','red','green','blue','cyan'};  % yellow is not clear to show
modul = mod(i,length(object_colors));
if (modul==0)
    modul = length(object_colors);
end
colors = object_colors{modul};
