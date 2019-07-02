library(ggplot2)
library(optparse)

se <- function(x)
{
    sd(x) / (sqrt(length(x)))
}

ci <- function(x)
{
    1.96 * se(x)
}

analysisPlot <- function(data_path, output_file = "analysis.png")
{
    data <- read.csv(data_path)
    print(names(data))
    values <- do.call(data.frame,
                      aggregate(validationScore~reader+optimizer+model,
                                data,
                                function(x) c(mean = mean(x),
                                              sd = sd(x),
                                              se = se(x),
                                              ci = ci(x))))
    values$group = paste(values$optimizer,values$model,sep='|')
    plotColumn <- "validationScore"
    meanStr <- sprintf("%s.mean", plotColumn)
    ciStr <- sprintf("%s.ci", plotColumn)
    yMinStr <- sprintf("%s-%s", meanStr, ciStr)
    yMaxStr <- sprintf("%s+%s", meanStr, ciStr)
    print("initing")
    g <- ggplot(values,
                aes_string(x="reader", y=meanStr,
                           ymin = yMinStr, ymax = yMaxStr,
                           group="group",
                           color="group",
                           fill="group"))
    print("init OK")
    g <- g + geom_ribbon(size=0,alpha = 0.3)
    g <- g + geom_point(size=2)
    g <- g + geom_line(size=0.5)
    g <- g + theme_bw()
    g <- g + labs(x = "nb samples by tag")
    g <- g + scale_x_log10(breaks=unique(data$reader))
#    g <- g + coord_cartesian(ylim=c(-15,-5))
    ggsave(output_file)
}

parametersPlot <- function(data_path, output_file = "parameters_analysis.png")
{
    data <- read.csv(data_path)
    # Plot data is not equivalent to data
    plotData <- data.frame(reader = integer(),
                           optimizer = character(),
                           param = character(),
                           value = double(),
                           sd = double(),
                           ci = double(),
                           stringsAsFactors=FALSE)
    print(head(data))
    # For each parameter (format is hard coded)
    print(names(data))
    for (i in 5:length(names(data)))
    {
        colname <- names(data)[i]
        print(paste0("treating ", colname))
        tmpData <- do.call(data.frame,aggregate(formula(paste0(colname,"~iteration_nb+optimizer")),
                                                data,
                                                function(x) c(mean = mean(x),
                                                              sd = sd(x),
                                                              ci = ci(x))))
        nrow <- nrow(tmpData)
        tmpData$param <- rep(colname, nrow)
        tmpData$value <- tmpData[,paste0(colname,".mean")]
        tmpData$sd <- tmpData[,paste0(colname,".sd")]
        tmpData$ci <- tmpData[,paste0(colname,".ci")]
        tmpData <- tmpData[,c("param","value","sd","ci","iteration_nb","optimizer")]
        plotData <- rbind(plotData,tmpData)
    }
    print(plotData)
    g <- ggplot(plotData, aes_string(x="iteration_nb",
                                     y="value",
                                     ymin="value - ci",
                                     ymax="value + ci",
                                     color="optimizer",
                                     fill="optimizer",
                                     group="interaction(param,optimizer)"))
    g <- g + geom_ribbon(alpha=0.5)
    g <- g + geom_ribbon(data=plotData,
                         mapping=aes_string(x="iteration_nb",
                                            y="value",
                                            ymin="value - sd",
                                            ymax="value + sd",
                                            color="optimizer",
                                            fill="optimizer",
                                            group="interaction(param,optimizer)"),
                         alpha=0.2)
    g <- g + geom_point()
    g <- g + geom_line()
    g <- g + facet_wrap(~param, scales="free_y",ncol=3)
    g <- g + scale_x_log10(breaks=unique(data$reader))
    g <- g + theme_bw()
    ggsave(output_file,width=16,height=40)
}

# Script OPTIONS
option_list <- list(
    make_option(c("-m","--mode"), type="character", default="validation",
                help="Choose graph mode [parameters|validation]")
)
parser <- OptionParser(usage="%prog [options] <file>",
                       option_list = option_list)

args <- commandArgs(TRUE)

# Read Options
cmd <- parse_args(parser, args, positional_arguments=1)

mode <- cmd$options$mode
input_file <- cmd$args[1]

if (mode == "parameters") {
    parametersPlot(input_file)
} else if (mode == "validation") {
    analysisPlot(input_file)
} else {
    print(paste("Unknown mode: '", mode , "'", sep=""))
}
