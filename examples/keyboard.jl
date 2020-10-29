function monitorInput()
    # Put STDIN in 'raw mode'
    ccall(:jl_tty_set_mode, Int32, (Ptr{Void}, Int32), STDIN.handle, true) == 0 || throw("FATAL: Terminal unable to enter raw mode.")

    inputBuffer = Channel{Char}(100)

    @async begin
        while true
            c = read(STDIN, Char)
            put!(inputBuffer, c)
        end
    end
    return inputBuffer
end


inputBuffer = monitorInput()
for i in 1:100
    if isready(inputBuffer) && take!(inputBuffer) == 'q'
        break
    end
    print('\r', i)
    sleep(0.1)
end

println("\nPlease have a nice day!")
