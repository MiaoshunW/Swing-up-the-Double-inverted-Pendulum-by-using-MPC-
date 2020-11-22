%
% File: DoublePendulum.m
%
classdef DoublePendulum < handle
  % A class for implementing a MATLAB Graphics visualization of the
  % Simulink model of the double pendulum which Guy Rouleau shared in
  % File Exchange posting 23126.
  %
  properties (SetAccess=private)
    Arms    = gobjects(1,2);
    Traces  = gobjects(1,2);
    Lengths = [0.17 0.314];
    Angles  = [0 0];
    BallWidth = .08;
    ArmWidth = .03;
  end
  %
  % Public methods
  methods
    function obj = DoublePendulum()
      obj.createGeometry();
      obj.updateTransforms();
    end
    function setAngles(obj, a1, a2)
    % Call this to change the angles.
      obj.Angles(1) = a1;
      obj.Angles(2) = a2;
      %obj.addTracePoints();
      obj.updateTransforms();
    end
%     function clearPoints(obj)
%     % Call this to reset the traces.
%       obj.Traces(1).clearpoints();
%       obj.Traces(2).clearpoints();
%     end
    function r = isAlive(obj)
    % Call this to check whether the figure window is still alive.
      r = isvalid(obj) && ...
          isvalid(obj.Arms(1)) && isvalid(obj.Arms(2)) && ...
          isvalid(obj.Traces(1)) && isvalid(obj.Traces(2));
    end
  end
  %
  % Private methods
  methods (Access=private)
    function createArm(obj, p, len, col)
    % Creates the geometry for one pendulum. This is basically a copy
    % of the function we created earlier.
      w = obj.ArmWidth;
      l = rectangle('Parent',p);
      l.Position = [-w/2, -len, w, len];
      l.FaceColor = col;
      l.EdgeColor = 'none';
      c = rectangle('Parent',p);
      r = obj.BallWidth;
      c.Position = [-r/2, -(len+r/2), r, r];
      c.Curvature = [1 1];
      c.EdgeColor = 'none';
      c.FaceColor = col;
    end
%     function addTracePoints(obj)
%     % Adds the current end points of the two pendulums to the traces.
%       a1 = obj.Angles(1);
%       a2 = obj.Angles(2);
%       l1 = obj.Lengths(1);
%       l2 = obj.Lengths(2);
%       x1 =  l1*sin(a1);
%       y1 = -l1*cos(a1);
%       x2 = x1 + l2*sin(a1+a2);
%       y2 = y1 - l2*cos(a1+a2);
%       obj.Traces(1).addpoints(x1,y1);
%       obj.Traces(2).addpoints(x2,y2);
%     end
    function createGeometry(obj)
    % Creates all of the graphics objects for the visualization.
      col1 = 'red';
      col2 = 'green';
      col3 = 'blue';
      fig = figure;
      ax = axes('Parent',fig);
      % Create the traces
%       obj.Traces(1) = animatedline('Parent', ax, 'Color', col1);
%       obj.Traces(2) = animatedline('Parent', ax, 'Color', col2);
      % Create the transforms
      obj.Arms(1) = hgtransform('Parent', ax);
      obj.Arms(2) = hgtransform('Parent', obj.Arms(1));
      % Create the arms
      createArm(obj, obj.Arms(1), obj.Lengths(1), col1);
      createArm(obj, obj.Arms(2), obj.Lengths(2), col2);
      % Create a blue circle at the origin.
      c = rectangle('Parent',ax);
      r = obj.BallWidth;
      c.Position = [-r/2, -r/2, r, r];
      c.Curvature = [1 1];
      c.EdgeColor = 'none';
      c.FaceColor = col3;
      % Initialize the axes.
      maxr = sum(obj.Lengths);
      ax.DataAspectRatio = [1 1 1];
      ax.XLim = [-1 1];
      ax.YLim = [-0.6 0.6];
      grid(ax,'on');
      ax.SortMethod = 'childorder';
    end
    function updateTransforms(obj)
    % Updates the transform matrices.
      a1 = obj.Angles(1);
      a2 = obj.Angles(2);
      offset = [0 -obj.Lengths(1) 0];
      pause(1);
      obj.Arms(1).Matrix = makehgtform('zrotate', a1);
      pause(1);
      obj.Arms(2).Matrix = makehgtform('translate', offset, ...
                                       'zrotate', a2);
    end
  end
end

