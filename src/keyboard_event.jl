"""
Read keyboard event.
"""
function monitorInput()
    # Put STDIN in 'raw mode'
    ccall(:jl_tty_set_mode, Int32, (Ptr{}, Int32), stdin.handle, true) == 0 || throw("FATAL: Terminal unable to enter raw mode.")

    inputBuffer = Channel{Char}(100)

    task = @async begin
        while true
            c = read(stdin, Char)
            put!(inputBuffer, c)
        end
    end
    return inputBuffer, task
end
