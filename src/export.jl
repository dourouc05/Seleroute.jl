function _export_lp_if_allowed(rd::RoutingData, m::Model, fn::AbstractString)
    if rd.export_lps
        write_LP("$(rd.output_folder)/$(fn).lp", m)
    end
end

function _export_lp_if_failed(rd::RoutingData, status::MOI.TerminationStatusCode, m::Model, fn::AbstractString, errmsg::AbstractString)
    if status != MOI.OPTIMAL
        if rd.export_lps_on_error
            try
                write_LP("$(rd.output_folder)/$(fn).lp", m)
            catch
                write_MOF("$(rd.output_folder)/$(fn).json", m)
            end
        end

        rd.logmessage("$(errmsg) Error code: $(status)")
        error("$(errmsg) Error code: $(status)")
    end
end
