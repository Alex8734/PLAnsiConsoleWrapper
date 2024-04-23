:- set_prolog_stack(global,limit(50 000 000 000)).
:- set_prolog_stack(trail, limit(20 000 000 000)).
:- set_prolog_stack(local, limit(2 000 000 000)).

symbol('██').
size(500,800).
offset(0, 6).
planes(0.1,10).

startAnim(X,F) :-
    start(X,F),
    NextX is X + 5,
    sleep(0.33),
    startAnim(NextX,F).

start(RotX,F) :-
    clear,
    read_obj_file(F,X,Y),
    transformMatrix(0, 0, -4, T),
    getRotationMatrixDegrees(0, 0, 90, R),
    scaleMatrix(1, 1, 1, S),
    fullMatrix(T, R, S, M),
    drawTriangles(X, Y, M).

fullMatrix(TransformMatrix, RotationMatrix, ScaleMatrix, Result) :-
    size(Height, Width),
    Aspect is Width / Height,
    planes(Near, Far),
    perspectiveMatrix(Aspect, 90, Near, Far, P),
    multiplyMatrices(P, TransformMatrix, PT),
    multiplyMatrices(PT, RotationMatrix, PTR),
    multiplyMatrices(PTR, ScaleMatrix, Result).

scaleMatrix(X, Y, Z, [
    [A, B, C, D],
    [E, F, G, H],
    [I, J, K, L],
    [M, N, O, P]
    ]) :-
    A is X, B is 0, C is 0, D is 0,
    E is 0, F is Y, G is 0, H is 0,
    I is 0, J is 0, K is Z, L is 0,
    M is 0, N is 0, O is 0, P is 1.

transformMatrix(X, Y, Z, [
    [A, B, C, D],
    [E, F, G, H],
    [I, J, K, L],
    [M, N, O, P]
    ]) :-
    A is 1, B is 0, C is 0, D is X,
    E is 0, F is 1, G is 0, H is Y,
    I is 0, J is 0, K is 1, L is Z,
    M is 0, N is 0, O is 0, P is 1.

getRotationMatrixDegrees(X, Y, Z, Matrix) :-
    toRad(X, RadX),
    toRad(Y, RadY),
    toRad(Z, RadZ),
    rotationMatrix(RadX, RadY, RadZ, Matrix).

rotationMatrix(X, Y, Z, Matrix) :-
    A is cos(X), B is -sin(X), C is sin(X), D is cos(X),
    E is cos(Y), F is sin(Y), G is -sin(Y), H is cos(Y),
    multiplyMatrices([
        [1, 0, 0, 0],
        [0, A, B, 0],
        [0, C, D, 0],
        [0, 0, 0, 1]
    ],[
        [E, 0, F, 0],
        [0, 1, 0, 0],
        [G, 0, H, 0],
        [0, 0, 0, 1]
    ], RotationXY),
    I is cos(Z), J is -sin(Z), K is sin(Z), L is cos(Z),
    multiplyMatrices(RotationXY, [
        [I, J, 0, 0],
        [K, L, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], Matrix).

perspectiveMatrix(Aspect, FOV, Near, Far, [
    [A, B, C, D],
    [E, F, G, H],
    [I, J, K, L],
    [M, N, O, P]
    ]) :-
    toRad(FOV, FOVRad),
    Tangent is tan(FOVRad/2),
    A is 1 / (Aspect * Tangent), B is 0, C is 0, D is 0,
    E is 0, F is 1 / Tangent, G is 0, H is 0,
    I is 0, J is 0, K is (Far + Near) / (Near - Far), L is (2 * Far * Near) / (Near - Far),
    M is 0, N is 0, O is -1, P is 0.

resetText :- write("\033[0").
clear :-
    write("\033[2J"),
    setCursorPosition(0,0).
setForegroundColor(R,G,B) :- format("\033[38;2;~w;~w;~wm", [R,G,B]).
setBackgroundColor(R,G,B) :- format("\033[48;2;~w;~w;~wm", [R,G,B]).
setCursorPosition(X, Y) :- format("\033[~w;~wH", [Y,X]).

drawPixel([X, Y], Symbol) :-
    X2 is X * 2,
    setCursorPosition(X2, Y),
    format("~w", [Symbol]),
    setCursorPosition(0,0).

drawLine([X,Y,Z], [X,Y,_], Symbol) :- drawFragmentIfNeeded([X, Y, Z], Symbol),!.
drawLine([X0,Y0,Z], [X1,Y1,_], Symbol) :-
    RoundedX0 is round(X0),
    RoundedY0 is round(Y0),
    drawFragmentIfNeeded([RoundedX0, RoundedY0, Z], Symbol),
    DistanceX is X1 - X0,
    DistanceY is Y1 - Y0,
    Distance is max(abs(DistanceX), abs(DistanceY)),
    NewX is X0 + DistanceX / Distance,
    NewY is Y0 + DistanceY / Distance,
    roundSmallFloatingPoint(NewX, RoundedNewX),
    roundSmallFloatingPoint(NewY, RoundedNewY),
    drawLine([RoundedNewX,RoundedNewY,Z], [X1,Y1,Z], Symbol).

drawFragmentIfNeeded([X,Y,Z], Symbol) :-
    (
    size(Height, Width),
    offset(OffsetX, OffsetY),

    X >= OffsetX, X =< Width + OffsetX,
    Y >= OffsetY, Y =< Height + OffsetY,
    Z >= -1, Z =< 1,

    drawPixel([X, Y], Symbol)
    ),!.

roundSmallFloatingPoint(X, Result) :-
    (abs(X - floor(X)) < 0.01,
    Result is floor(X),!);
    Result is X.

drawTriangles(_, [], _).
drawTriangles(Vertices, [[X,Y,Z]|Tail], Matrix) :-
    symbol(Symbol),

    getElement(Vertices, X, Vertex1),
    getElement(Vertices, Y, Vertex2),
    getElement(Vertices, Z, Vertex3),

    vertexShader(Matrix, Vertex1, NewVertex1),
    vertexShader(Matrix, Vertex2, NewVertex2),
    vertexShader(Matrix, Vertex3, NewVertex3),

    projectionDivision(NewVertex1, NormalizedVertex1),
    projectionDivision(NewVertex2, NormalizedVertex2),
    projectionDivision(NewVertex3, NormalizedVertex3),

    drawTriangleIfNeeded(NormalizedVertex1, NormalizedVertex2, NormalizedVertex3, Symbol),

    drawTriangles(Vertices, Tail, Matrix).

drawTriangleIfNeeded(Vertex1, Vertex2, Vertex3, Symbol) :-
    (
    shouldShow(Vertex1, Vertex2, Vertex3),

    scaleToScreen(Vertex1, ScreenVertex1),
    scaleToScreen(Vertex2, ScreenVertex2),
    scaleToScreen(Vertex3, ScreenVertex3),

    drawLine(ScreenVertex1, ScreenVertex2, Symbol),
    drawLine(ScreenVertex2, ScreenVertex3, Symbol),
    drawLine(ScreenVertex3, ScreenVertex1, Symbol));!.

shouldShow([X0, Y0, _], [X1, Y1, _], [X2, Y2, _]) :-
    A is X0*Y1 - X1*Y0 + X1*Y2 - X2*Y1 + X2*Y0 - X0*Y2,
    A >= 0.

getElement([Head|_], 0, Head).
getElement([_|Tail], Index, Result) :-
    NextIndex is Index - 1,
    getElement(Tail, NextIndex, Result).

projectionDivision([X,Y,Z,W], [NewX,NewY,NewZ]) :-
    NewX is X / W,
    NewY is Y / W,
    NewZ is Z / -W.

scaleToScreen([X,Y,Z], [NewX,NewY,Z]) :-
    size(Height, Width),
    offset(OffsetX, OffsetY),
    NewX is round((X + 1) / 2 * Width + OffsetX),
    NewY is round(Height - (Y + 1) / 2 * Height + OffsetY).

vertexShader(Matrix, [X,Y,Z], [NewX,NewY,NewZ,W]) :-
    multiplyMatrixWithVector(Matrix, [X,Y,Z,1], [NewX,NewY,NewZ,W]).

toRad(X, Y) :- Y is X * pi / 180.

multiplyVector([X0,Y0], [X1,Y1], [X2,Y2]) :-
    X2 is X0 * X1,
    Y2 is Y0 * Y1.

multiplyVector([X0,Y0], Factor, [X1,Y1]) :-
    X1 is X0 * Factor,
    Y1 is Y0 * Factor.

multiplyMatrixWithVector([
    [A,B,C,D],
    [E,F,G,H],
    [I,J,K,L],
    [M,N,O,P]
    ],
    [X,Y,Z,W],
    [ResultX,ResultY,ResultZ,ResultW]) :-
    ResultX is X*A + Y*B + Z*C + W*D,
    ResultY is X*E + Y*F + Z*G + W*H,
    ResultZ is X*I + Y*J + Z*K + W*L,
    ResultW is X*M + Y*N + Z*O + W*P.

multiplyMatrices([
    [X0,Y0,Z0,W0],
    [X1,Y1,Z1,W1],
    [X2,Y2,Z2,W2],
    [X3,Y3,Z3,W3]],
    [
    [A0,B0,C0,D0],
    [A1,B1,C1,D1],
    [A2,B2,C2,D2],
    [A3,B3,C3,D3]
    ],[
    [ResultX0,ResultY0,ResultZ0,ResultW0],
    [ResultX1,ResultY1,ResultZ1,ResultW1],
    [ResultX2,ResultY2,ResultZ2,ResultW2],
    [ResultX3,ResultY3,ResultZ3,ResultW3]]) :-
    ResultX0 is X0*A0 + Y0*A1 + Z0*A2 + W0*A3,
    ResultY0 is X0*B0 + Y0*B1 + Z0*B2 + W0*B3,
    ResultZ0 is X0*C0 + Y0*C1 + Z0*C2 + W0*C3,
    ResultW0 is X0*D0 + Y0*D1 + Z0*D2 + W0*D3,

    ResultX1 is X1*A0 + Y1*A1 + Z1*A2 + W1*A3,
    ResultY1 is X1*B0 + Y1*B1 + Z1*B2 + W1*B3,
    ResultZ1 is X1*C0 + Y1*C1 + Z1*C2 + W1*C3,
    ResultW1 is X1*D0 + Y1*D1 + Z1*D2 + W1*D3,

    ResultX2 is X2*A0 + Y2*A1 + Z2*A2 + W2*A3,
    ResultY2 is X2*B0 + Y2*B1 + Z2*B2 + W2*B3,
    ResultZ2 is X2*C0 + Y2*C1 + Z2*C2 + W2*C3,
    ResultW2 is X2*D0 + Y2*D1 + Z2*D2 + W2*D3,

    ResultX3 is X3*A0 + Y3*A1 + Z3*A2 + W3*A3,
    ResultY3 is X3*B0 + Y3*B1 + Z3*B2 + W3*B3,
    ResultZ3 is X3*C0 + Y3*C1 + Z3*C2 + W3*C3,
    ResultW3 is X3*D0 + Y3*D1 + Z3*D2 + W3*D3.

% to read an obj file

read_obj_file(File) :-
    open(File, read, Stream),
    read_obj_lines(Stream, 0, [], [], Vertices, Faces),
    close(Stream),
    write(Faces),
    write(Vertices).

read_obj_file(File, Vertices, Faces) :-
    open(File, read, Stream),
    read_obj_lines(Stream, 0, [], [], Vertices, Faces),
    close(Stream).

read_obj_lines(Stream, Count, VTemp, NTemp, Vertices, Faces) :-
    at_end_of_stream(Stream), !,
    reverse(VTemp, Vertices),
    reverse(NTemp, Faces),
    format('Lines read: ~w~n', [Count]).

read_obj_lines(Stream, Count, VTemp, NTemp, Vertices, Normals) :-
    \+ at_end_of_stream(Stream),
    read_line_to_string(Stream, Line),
    split_string(Line, " ", "", [_|LineParts]),
    NewCount is Count + 1,
    ( startsWith(Line, 'v') -> filter_vertices(LineParts, VertParts), VNew = [VertParts|VTemp], NNew = NTemp;
      startsWith(Line, 'f') -> filter_faces(LineParts,Faces ), NNew = [Faces|NTemp], VNew = VTemp;
      VNew = VTemp, NNew = NTemp
    ),
    read_obj_lines(Stream, NewCount, VNew, NNew, Vertices, Normals).

filter_faces([], []).
filter_faces([H|T], [First|Result]) :-
    split_string(H, "/", "", [Temp|_]),
    atom_number(Temp, TempNumber),
    First is TempNumber - 1,
    filter_faces(T, Result).

filter_vertices([], []).
filter_vertices([H|T], [First|Result]) :-
    atom_number(H, First),
    filter_vertices(T, Result).


startsWith(String, Letter) :-
    atom_chars(String, [H|_]),
    H = Letter.
